package org.firstinspires.ftc.teamcode.hardware.ruckus;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.CameraSystem;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.components.SwerveChassis;
import org.firstinspires.ftc.teamcode.opmodes.ruckus.CenaPlayer;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
import org.firstinspires.ftc.teamcode.support.diagnostics.MenuEntry;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;

public class ToboRuckus extends Logger<ToboRuckus> implements Robot {
    private Telemetry telemetry;
    public Configuration configuration;
    public SwerveChassis chassis;
    public MineralIntake intake;
    public MineralDelivery mineralDelivery;
    public Hanging hanging;
    public CameraSystem cameraSystem;
    public CameraMineralDetector cameraMineralDetector;
    public CoreSystem core;
    public ElapsedTime runtime = new ElapsedTime();
    public double rotateRatio = 0.7; // slow down ratio for rotation
    public double motor_count = 0;
    public boolean playingSong = false;

    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    @Override
    public void configure(Configuration configuration, Telemetry telemetry, boolean auto) {
        runtime.reset();
        double ini_time = runtime.seconds();
        this.telemetry = telemetry;
        this.configuration = configuration;

//        cameraSystem = new CameraSystem(null);
//        cameraSystem.init(configuration.getHardwareMap());
        if (auto) {
            cameraMineralDetector = new CameraMineralDetector().configureLogging("CameraMineralDetector", logLevel);
            cameraMineralDetector.configure(configuration, true);
        }
        this.core = new CoreSystem();
        info("RoboRuck configure() after new CoreSystem()(run time = %.2f sec)", (runtime.seconds() - ini_time));
        chassis = new SwerveChassis(this.core).configureLogging("Swerve", logLevel); // Log.DEBUG
        chassis.configure(configuration, auto, true);
        info("RoboRuck configure() after init Chassis (run time = %.2f sec)", (runtime.seconds() - ini_time));
        intake = new MineralIntake().configureLogging("Intake", logLevel);
        intake.configure(configuration);
        info("RoboRuck configure() after init Intake (run time = %.2f sec)", (runtime.seconds() - ini_time));
        hanging = new Hanging(this.core).configureLogging("Hanging", logLevel);
        hanging.configure(configuration, auto, chassis.orientationSensor);
        info("RoboRuck configure() after init Hanging (run time = %.2f sec)", (runtime.seconds() - ini_time));
        mineralDelivery = new MineralDelivery().configureLogging("Delivery", logLevel);
        mineralDelivery.configure(configuration);
        info("RoboRuck configure() after init Delivery (run time = %.2f sec)", (runtime.seconds() - ini_time));
    }

    public void AutoRoutineTest() throws InterruptedException {
        chassis.driveAndSteerAuto(0.6, 560 * 3, 45);
    }

    @Override
    public void reset(boolean auto) {
        chassis.reset();
        intake.reset(auto);
        mineralDelivery.reset();
        hanging.reset(auto);
        if (auto) {
            chassis.setupTelemetry(telemetry);
        }
    }

    @MenuEntry(label = "TeleOp", group = "Competition")
    public void mainTeleOp(EventManager em, EventManager em2) {
        telemetry.addLine().addData("(RS)", "4WD").setRetained(true)
                .addData("(RS) + (LS)", "2WD / Steer").setRetained(true);
        telemetry.addLine().addData("< (LS) >", "Rotate").setRetained(true)
                .addData("[LB]/[LT]", "Slow / Fast").setRetained(true);
        telemetry.addLine().addData("motor_count=", new Func<String>() {
            @Override
            public String value() {
                return String.format("%2.0f", motor_count);
            }
        });
        chassis.setupTelemetry(telemetry);
        intake.setupTelemetry(telemetry);
        hanging.setupTelemetry(telemetry);
        mineralDelivery.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
//        if (!hanging.latchIsBusy()) {
//            hanging.resetLatch();
//        }

        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.BOTH)) < 0.1) {
                    // right stick with idle left stick operates robot in "crab" mode
                    double power = Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.BOTH));
                    power *= power; // square power to stick
                    double heading = toDegrees(currentX, currentY);
                    double cur_heading = chassis.getCurHeading();
                    // invert headings less than -90 / more than 90
                    if ((Math.abs(cur_heading - heading) < 10) || (Math.abs(currentX) + Math.abs(currentY) < 0.1)) { // keep original heading
                        heading = cur_heading;
                    }
                    // dead zone mapping: [-120, -75] to -90
                    // dead zone mapping: [75, 120] to 90
                    if (heading>-120 && heading<-75) heading = -90;
                    if (heading>75 && heading<120) heading = 90;
                    if ((Math.abs(cur_heading-heading)==180) && Math.abs(heading)<=90) {
                        heading = cur_heading;
                        power = -1 * power;
                    }

                    if (Math.abs(heading) > 90) { // reduce rotation angle
                        heading -= Math.signum(heading) * 180;
                        power = -1 * power;
                    }

                    debug("sticksOnly(): straight, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, true);
                } else {
                    // right stick with left stick operates robot in "car" mode
                    double heading = source.getStick(Events.Side.LEFT, Events.Axis.X_ONLY) * 90;
                    double power = currentY * Math.abs(currentY);
                    debug("sticksOnly(): right / steer, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, false);
                }
            }
        }, Events.Axis.BOTH, Events.Side.RIGHT);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.BOTH)) < 0.2) {
                    // left stick with idle right stick rotates robot in place
                    chassis.rotate(currentX * Math.abs(currentX) * powerAdjustment(source)*rotateRatio);
                } else if (source.getTrigger(Events.Side.RIGHT) < 0.2) {
                    // right stick with left stick operates robot in "car" mode
                    double heading = currentX * 90;
                    double power = source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY);
                    debug("sticksOnly(): left / steer, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, false);
                }
                else {
                    chassis.stop();
                }
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);
//        em.onButtonDown(new Events.Listener() {
//            @Override
//            public void buttonDown(EventManager source, Button button) throws InterruptedException {
//                double heading = button==Button.DPAD_LEFT ? -90 : 90;
//                chassis.driveAndSteer(powerAdjustment(source), heading, true);
//            }
//        }, Button.DPAD_RIGHT, Button.DPAD_LEFT);
//        em.onButtonUp(new Events.Listener() {
//            @Override
//            public void buttonUp(EventManager source, Button button) throws InterruptedException {
//                chassis.driveAndSteer(0, 0, true);
//            }
//        }, Button.DPAD_RIGHT, Button.DPAD_LEFT);

        // Same as em2: DPAD LEFT / RIGHT operates slider. {Back] oerrides encoder limits
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (button == Button.DPAD_LEFT) { // slide out
                    if (source.isPressed(Button.BACK)) {
                        intake.adjustSlider(true);
                    } else if (intake.getSliderCurrent() >= intake.getSliderDump()) {
                        intake.moveSlider(intake.getSliderExtended());
                    } else {
                        intake.moveSlider(intake.getSliderDump());
                    }
                } else if (!source.isPressed(Button.BACK) && intake.isBoxDown() && (intake.getSliderCurrent() < intake.getSliderMinSweep())) {
                    ; // cannot move further when box is down
                } else if (button == Button.DPAD_RIGHT) { // slide in
                    if (source.isPressed(Button.BACK)) {
                        intake.adjustSlider(false);
                    } else if (intake.getSliderCurrent() >= intake.getSliderDump()) {
                        intake.moveSlider(intake.getSliderDump());
                    } else {
                        intake.moveSlider(intake.getSliderContracted());
                    }
                }
            }
        }, Button.DPAD_LEFT, Button.DPAD_RIGHT);
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) {
                if (intake.getSliderCurrent() >= intake.getSliderDump()) intake.stopSlider();
            }
        }, Button.DPAD_LEFT, Button.DPAD_RIGHT);

        // DPAD UP / DOWN operates intake box position
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (button == Button.DPAD_UP) {
                    if (source.isPressed(Button.BACK))
                        intake.moveBox(true, false);
                    else
                        intake.moveBox(true, true);
                } else {
                    intake.boxLiftDownCombo(); // ensure slider extended out before down
                }
            }
        }, Button.DPAD_UP, Button.DPAD_DOWN);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.RIGHT_BUMPER)) {
                    mineralDelivery.returnCombo();
                    intake.moveGate(false); // auto close gate when arm down
                    mineralDelivery.liftStop();
                } else if (!source.isPressed(Button.START)) {
                    intake.moveGate(!intake.isGateOpen());
                }
            }
        }, Button.B);

        // [X] opens / closes delivery gate
        // [RB] + [X] is delivery combo (move slider out, close gate, arm lift up, arm up)
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.RIGHT_BUMPER)) {
                    mineralDelivery.deliveryCombo(intake);
                } else {
                    mineralDelivery.gateAuto();
                }
            }
        }, Button.X);
        em.onTrigger(new Events.Listener() {
            @Override
            public void triggerMoved(EventManager source, Events.Side side, float current, float change) throws InterruptedException {
                // 0.2 is a dead zone threshold for the trigger
                /* if (current > 0.2) {
                    intake.rotateSweeper(MineralIntake.SweeperMode.PUSH_OUT);
                } else if (current == 0) {
                    intake.rotateSweeper(MineralIntake.SweeperMode.HORIZONTAL_STOP);
                }*/
                if (current > 0.2) {
                    if (intake.isBoxDown() || source.isPressed(Button.BACK))
                        intake.sweeperOut();
                } else if (current == 0) {
                    intake.stopSweeper();
                }
            }
        }, Events.Side.LEFT);
        em.onTrigger(new Events.Listener() {
            @Override
            public void triggerMoved(EventManager source, Events.Side side, float current, float change) throws InterruptedException {
                // 0.2 is a dead zone threshold for the trigger
                if (current > 0.2) {
                    if (playingSong) {
                        CenaPlayer.stop();
                    } else {
                        CenaPlayer.start(configuration.getHardwareMap().appContext, telemetry);
                    }
                    playingSong = !playingSong;
                }
            }
        }, Events.Side.RIGHT);
//        em.onTrigger(new Events.Listener() {
//            @Override
//            public void triggerMoved(EventManager source, Events.Side side, float current, float change) throws InterruptedException {
//                if (current>0.2) {
//                    double power = source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY);
//                    power *= Math.abs(power);
//                    if (power>1.0) power = 1.0;
//                    else if (power<-1.0) power = -1.0;
//                    double heading = source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY) * 90;
//                    chassis.driveAndSteer(power, heading, false);
//                } else if (current <= 0.2) {
//                    chassis.driveAndSteer(0, 0, true);
//                }
//            }
//        }, Events.Side.RIGHT);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                // intake.rotateSweeper(MineralIntake.SweeperMode.INTAKE);
                // intake.boxLiftDownCombo();
                if (!TaskManager.isComplete("mineralDump") && !source.isPressed(Button.BACK))
                    return;
                if (intake.getSliderCurrent() > intake.getSliderInitOut()) {
                    // automatically down
                    intake.boxDown();
                }
                if (intake.isBoxDown() || source.isPressed(Button.BACK)) {
                    intake.sweeperIn();
                }
                intake.moveGate(false); // auto close the gate when sweeping
            }
        }, Button.LEFT_BUMPER);
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                //intake.rotateSweeper(MineralIntake.SweeperMode.VERTICAL_STOP);
                intake.stopSweeper();
            }
        }, Button.LEFT_BUMPER);

        // DPAD LEFT / RIGHT operates slider. {Back] oerrides encoder limits
        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (button == Button.DPAD_LEFT) { // slide out
                    if (source.isPressed(Button.BACK)) {
                        intake.adjustSlider(true);
                    } else {
                        intake.moveSlider(intake.getSliderExtended());
                    }
                } else if (!source.isPressed(Button.BACK) && intake.isBoxDown() && (intake.getSliderCurrent() < intake.getSliderMinSweep())) {
                    ; // cannot move further when box is down
                } else if (button == Button.DPAD_RIGHT) { // slide in
                    if (source.isPressed(Button.BACK)) {
                        intake.adjustSlider(false);
                    } else {
                        intake.moveSlider(intake.getSliderContracted());
                    }
                }
            }
        }, Button.DPAD_LEFT, Button.DPAD_RIGHT);
        em2.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) {
                intake.stopSlider();
            }
        }, Button.DPAD_LEFT, Button.DPAD_RIGHT);

        // DPAD UP / DOWN operates intake box position
        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (button == Button.DPAD_UP) {
                    if (source.isPressed(Button.BACK))
                        intake.moveBox(true, false);
                    else
                        intake.moveBox(true, true);
                } else {
                    intake.boxLiftDownCombo(); // ensure slider extended out before down
                }
            }
        }, Button.DPAD_UP, Button.DPAD_DOWN);

        em2.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.RIGHT_BUMPER)) {
                    hanging.latchAuto();
                }
            }
        }, Button.LEFT_BUMPER);

        // [B] opens / closes intake box gate
        // [LB] + [B] is delivery return combo (close gate, arm down, arm lift down)
        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.LEFT_BUMPER)) {
                    mineralDelivery.returnCombo();
                    intake.moveGate(false); // auto close gate when arm down
                } else if (!source.isPressed(Button.START)) {
                    intake.moveGate(!intake.isGateOpen());
                }
            }
        }, Button.B);

        // [X] opens / closes delivery gate
        // [LB] + [X] is delivery combo (move slider out, close gate, arm lift up, arm up)
        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.LEFT_BUMPER)) {
                    mineralDelivery.deliveryCombo(intake);
                } else {
                    mineralDelivery.gateAuto();
                }
            }
        }, Button.X);

        // (LS) operates delivery lift. [Back] overrides encoder position limits
        em2.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) {
                if (source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY) > 0.2) {
                    mineralDelivery.liftUp(source.isPressed(Button.BACK));
                    mineralDelivery.gateClose(); // close dumper gate
                    intake.moveGate(false); // close intake box
                } else if (source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY) < -0.2) {
                    mineralDelivery.liftDown(source.isPressed(Button.BACK));
                } else {
                    mineralDelivery.liftStop();
                }
            }
        }, Events.Axis.Y_ONLY, Events.Side.LEFT);

        // (RS) operates delivery arm. With [RT] pressed changes are gradual
        // (RS) with [RB] pressed operates latch up / down. [Back] overrides encoder position limits
        em2.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) {
                if (source.isPressed(Button.RIGHT_BUMPER)) {
                    // operate latch
                    if (currentY > 0.2) {
                        hanging.latchUp(source.isPressed(Button.BACK));
                    } else if (currentY < -0.2) {
                        hanging.latchDown(source.isPressed(Button.BACK));
                    } else if (!hanging.latchIsBusy()) {
                        hanging.latchStop();
                    }
                    return;
                } else if (!hanging.latchIsBusy()) {
                    hanging.latchStop();
                }
                if (source.getTrigger(Events.Side.RIGHT) > 0.2) {
                    // operate delivery arm gradually
                    if (currentY > 0.2) {
                        mineralDelivery.armUpInc();
                    } else if (currentY < -0.2) {
                        mineralDelivery.armDownInc();
                    } else {
                        mineralDelivery.armStop();
                    }
                    return;
                }
                if (currentY > 0.95) {
                    mineralDelivery.armDumpAuto();
                } else if (currentY < -0.95) {
                    mineralDelivery.armDown();
                }
            }
        }, Events.Axis.Y_ONLY, Events.Side.RIGHT);

        // em: [RB] + [Y] for mineral dump combo (move slider to dump, intake box up, open box gate)
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.START) && source.isPressed(Button.BACK)) { // testing chassis speed
                    motor_count = chassis.driveStraightSec(1.0, 10);
                    return;
                } else if (source.isPressed(Button.LEFT_BUMPER) && source.isPressed(Button.RIGHT_BUMPER)) { // testing chassis speed
                    motor_count = chassis.driveStraightSec(1.0, 2);
                    return;
                }
                else if (source.isPressed(Button.BACK)) { // default scale up
                    chassis.setDefaultScale(0.8);
                    return;
                }
                if (!source.isPressed(Button.RIGHT_BUMPER)) return;
                mineralDelivery.armCollectPos();
                mineralDelivery.gateOpen();
                intake.mineralDumpCombo(mineralDelivery, false);
            }
        }, Button.Y);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.BACK)) { // default scale back to 0.5
                    chassis.setDefaultScale(0.5);
                }
            }
        }, Button.A);
        // em2: [LB] + [Y] for mineral dump combo (move slider to dump, intake box up, open box gate)
        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.BACK)) { // transfer + delivery
                    mineralDelivery.armCollectPos();
                    mineralDelivery.gateOpen();
                    intake.mineralDumpCombo(mineralDelivery, true);
                } else if (!source.isPressed(Button.LEFT_BUMPER)) {
                    mineralDelivery.wristUpInc();
                } else {
                    mineralDelivery.armCollectPos();
                    mineralDelivery.gateOpen();
                    intake.mineralDumpCombo(mineralDelivery, false);
                }
            }
        }, Button.Y);
        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.BACK)) {
                    intake.syncSliderEncoder(30);
                } else if (!source.isPressed(Button.START)) {
                    mineralDelivery.wristDownInc();
                }
            }
        }, Button.A);
    }

    @MenuEntry(label = "Drive Straight", group = "Test Chassis")
    public void testStraight(EventManager em) {
        telemetry.addLine().addData("(LS)", "Drive").setRetained(true)
                .addData("Hold [LB]/[RB]", "45 degree").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 1000);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) throws InterruptedException {
                double power = Math.max(Math.abs(currentX), Math.abs(currentY));
                double heading = toDegrees(currentX, currentY);
                debug("testStraight(): x: %+.2f, y: %+.2f, pow: %+.3f, head: %+.1f",
                        currentX, currentY, power, heading);

                if (source.isPressed(Button.LEFT_BUMPER) || source.isPressed(Button.RIGHT_BUMPER)) {
                    // constrain to 45 degree diagonals
                    heading = Math.signum(heading) * (Math.abs(heading) < 90 ? 45 : 135);
                } else {
                    // constrain to 90 degrees
                    heading = Math.round(heading / 90) * 90;
                }

                // adjust heading / power for driving backwards
                if (heading > 90) {
                    chassis.driveStraight(-1.0 * power, heading - 180);
                } else if (heading < -90) {
                    chassis.driveStraight(-1.0 * power, heading + 180);
                } else {
                    chassis.driveStraight(power, heading);
                }
            }
        }, Events.Axis.BOTH, Events.Side.LEFT);
    }

    @MenuEntry(label = "Rotate in Place", group = "Test Chassis")
    public void testRotate(EventManager em) {
        telemetry.addLine().addData(" < (LS) >", "Power").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                chassis.rotate(currentX);
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);
    }

    @MenuEntry(label = "Test Sample", group = "Test Auto")
    public void retrieveSample(ToboRuckus.MineralDetection.SampleLocation sam_loc) throws InterruptedException {
        switch (sam_loc) {
            case CENTER:
                chassis.driveStraightAuto(0.4, 40, -4, Integer.MAX_VALUE);
                break;
            case RIGHT:
                chassis.driveStraightAuto(0.4, 58, 45, Integer.MAX_VALUE);
                break;
            case LEFT:
                chassis.driveStraightAuto(0.4, 55, -50, Integer.MAX_VALUE);
                break;
            default: // go straight like center
                chassis.driveStraightAuto(0.4, 40, -4, Integer.MAX_VALUE);
        }
        sweepSample();
        //if (!Thread.currentThread().isInterrupted())
        //    Thread.sleep(100);
        chassis.driveStraightAuto(0.4, 7, 0, Integer.MAX_VALUE);
        //if (!Thread.currentThread().isInterrupted())
        //intake.rotateSweeper(MineralIntake.SweeperMode.VERTICAL_STOP);
    }

    public enum Side {
        GOLD, SILVER;
    }

    public void goParking(Side side, boolean DoCollection) throws InterruptedException {
        if (side == side.GOLD) {
            chassis.driveAlongTheWall(0.55, 125, 5, SwerveChassis.Wall.RIGHT, 4000);//power was 0.5
        } else {
            chassis.driveAlongTheWall(0.55, 130, 5, SwerveChassis.Wall.LEFT, 4000);//power was 0.5
        }
        if (DoCollection) {
            autoCollect(25, true);
        } else {
            extendInakeForParking(750);
            chassis.driveStraightAuto(0.17, 10, side == side.GOLD ? 10 : -10, 500);
        }
    }

    public void goStateParking(Side side) throws InterruptedException {
        chassis.driveStraightAuto(0.4, -90, 0, 1000);
        if (!Thread.currentThread().isInterrupted())
            Thread.sleep(100);
        chassis.driveStraightAuto(0.25, +20, +90, 1000);
        chassis.rotateTo(0.4, 135);
        double detectedRightDistance = chassis.distanceToRight();
        chassis.driveStraightAuto(0.4, detectedRightDistance - 12, +90, 2000);
        detectedRightDistance = chassis.distanceToRight();
        if (!Thread.currentThread().isInterrupted())
            Thread.sleep(100);
        chassis.driveStraightAuto(0.3, detectedRightDistance - 12, +90, 2000);
        chassis.driveStraightAuto(0.4, 100, 0, 2000);
    }

    @MenuEntry(label = "Test Sample", group = "Test Auto")
    public void alignWithWallsState(ToboRuckus.MineralDetection.SampleLocation sam_loc) throws InterruptedException {
        //drive to default start position for gold side
        chassis.driveStraightAuto(0.4, 20, 0, Integer.MAX_VALUE);
        if (!Thread.currentThread().isInterrupted())
            Thread.sleep(100);
        //rotate robot parallel to the walls
//        telemetry.addLine("imu heading:%");
        chassis.rotateTo(0.4, -45);
//        chassis.rotateTo(0.18, 135);

        //from here, three different routine will converge into the depot
        if (!Thread.currentThread().isInterrupted())
            Thread.sleep(100);

        //align with right wall

        double detectedLeftDistance = chassis.distanceToLeft();
        switch (sam_loc) {
            case CENTER: // center
                detectedLeftDistance = Math.min(60, detectedLeftDistance);
                break;
            case RIGHT:
                detectedLeftDistance = Math.min(100, detectedLeftDistance);
                break;
            case LEFT:
                detectedLeftDistance = Math.min(20, detectedLeftDistance);
                break;
            default: // go straight like center
                detectedLeftDistance = Math.min(60, detectedLeftDistance);
        }
        chassis.driveStraightAuto(0.4, detectedLeftDistance - 12, -90, 2000);
        detectedLeftDistance = chassis.distanceToLeft();
        if (!Thread.currentThread().isInterrupted())
            Thread.sleep(100);
        chassis.driveStraightAuto(0.3, detectedLeftDistance - 12, -90, 2000);
//        telemetry.addLine(String.format("adjusted distance to right: %.3f",chassis.distanceToRight()));
//        telemetry.update();

        //force heading correction
        if (!Thread.currentThread().isInterrupted())
            Thread.sleep(100);
//        chassis.rotateTo(0.18, 138);

        //align with back wall
//        telemetry.addLine(String.format("detected distance to back: %.3f",chassis.distanceToBack()));
//        telemetry.update();
        double detectedFrontDistance = chassis.distanceToFront();
        switch (sam_loc) {
            case CENTER: // center
                detectedFrontDistance = Math.min(50, detectedFrontDistance);
                break;
            case RIGHT:
                detectedFrontDistance = Math.min(20, detectedFrontDistance);
                break;
            case LEFT:
                detectedFrontDistance = Math.min(80, detectedFrontDistance);
                break;
            default: // go straight like center
                detectedFrontDistance = Math.min(50, detectedFrontDistance);
        }

        chassis.driveStraightAuto(0.4, detectedFrontDistance - 50.0, 0, 1500);

        if (!Thread.currentThread().isInterrupted())
            Thread.sleep(100);
        detectedFrontDistance = chassis.distanceToBack();
//        telemetry.addLine(String.format("distance to back: %.3f",detectedFrontDistance));
//        telemetry.update();
//        Thread.sleep(1000);
        chassis.driveStraightAuto(0.3, detectedFrontDistance - 50.0, 0, 1500);

    }

    @MenuEntry(label = "Test Sample", group = "Test Auto")
    public void alignWithWallsGoldSide(ToboRuckus.MineralDetection.SampleLocation sam_loc) throws InterruptedException {
        //drive to default start position for gold side
        chassis.driveStraightAuto(0.4, 20, 0, Integer.MAX_VALUE);
        if (!Thread.currentThread().isInterrupted())
            Thread.sleep(100);
        //rotate robot parallel to the walls
//        telemetry.addLine("imu heading:%");
        chassis.rotateTo(0.4, 135);
//        chassis.rotateTo(0.18, 135);

        //from here, three different routine will converge into the depot
        if (!Thread.currentThread().isInterrupted())
            Thread.sleep(100);

        //align with right wall

        double detectedRightDistance = chassis.distanceToRight();
        switch (sam_loc) {
            case CENTER: // center
                detectedRightDistance = Math.min(60, detectedRightDistance);
                break;
            case RIGHT:
                detectedRightDistance = Math.min(100, detectedRightDistance);
                break;
            case LEFT:
                detectedRightDistance = Math.min(20, detectedRightDistance);
                break;
            default: // go straight like center
                detectedRightDistance = Math.min(60, detectedRightDistance);
        }
        chassis.driveStraightAuto(0.4, detectedRightDistance - 12, +90, 2000);
        detectedRightDistance = chassis.distanceToRight();
        if (!Thread.currentThread().isInterrupted())
            Thread.sleep(100);
        chassis.driveStraightAuto(0.3, detectedRightDistance - 12, +90, 2000);
//        telemetry.addLine(String.format("adjusted distance to right: %.3f",chassis.distanceToRight()));
//        telemetry.update();

        //force heading correction
        if (!Thread.currentThread().isInterrupted())
            Thread.sleep(100);
//        chassis.rotateTo(0.18, 138);

        //align with back wall
//        telemetry.addLine(String.format("detected distance to back: %.3f",chassis.distanceToBack()));
//        telemetry.update();
        double detectedBackDistance = chassis.distanceToBack();
        switch (sam_loc) {
            case CENTER: // center
                detectedBackDistance = Math.min(50, detectedBackDistance);
                break;
            case RIGHT:
                detectedBackDistance = Math.min(20, detectedBackDistance);
                break;
            case LEFT:
                detectedBackDistance = Math.min(80, detectedBackDistance);
                break;
            default: // go straight like center
                detectedBackDistance = Math.min(50, detectedBackDistance);
        }


        chassis.driveStraightAuto(0.4, 50.0 - detectedBackDistance, 0, 1500);

        if (!Thread.currentThread().isInterrupted())
            Thread.sleep(100);
        detectedBackDistance = chassis.distanceToBack();
//        telemetry.addLine(String.format("distance to back: %.3f",detectedBackDistance));
//        telemetry.update();
//        Thread.sleep(1000);
        chassis.driveStraightAuto(0.3, 50.0 - detectedBackDistance, 0, 1500);

    }

    public void collectSampleAndGoToWall(ToboRuckus.MineralDetection.SampleLocation sam_loc, Side side) throws InterruptedException {
        switch (sam_loc) {//Gold:Silver
            case CENTER: // center
                chassis.rotateTo(0.4, side == Side.GOLD ? -90 : -91);
                autoCollect(20, false);
                break;
            case RIGHT:
                chassis.rotateTo(0.4, side == Side.GOLD ? -61 : -62);
                autoCollect(28, false);
                break;
            case LEFT:
                chassis.rotateTo(0.4, side == Side.GOLD ? -125 : -126);
                autoCollect(28, false);
                break;
            default: // go straight like center
                chassis.rotateTo(0.4, side == Side.GOLD ? -90 : -91);
                autoCollect(20, false);
        }
        autoTransfer();
        if (sam_loc != ToboRuckus.MineralDetection.SampleLocation.CENTER)
            chassis.rotateTo(0.4, -90);

        chassis.driveStraightAuto(.4, 5, 0, 1000);
        chassis.driveStraightAuto(0.4, 90, -70, 3000);
        Thread.sleep(100);
    }

    public void collectSampleAndDeliver(ToboRuckus.MineralDetection.SampleLocation sam_loc) throws InterruptedException {
        switch (sam_loc) {//Gold:Silver
            case CENTER: // center
                chassis.rotateTo(0.4, -90);
                autoCollect(20, false);
                break;
            case RIGHT:
                chassis.rotateTo(0.4, -61);
                autoCollect(28, false);
                break;
            case LEFT:
                chassis.rotateTo(0.4, -125);
                autoCollect(28, false);
                break;
            default: // go straight like center
                chassis.rotateTo(0.4, -90);
                autoCollect(20, false);
        }
        autoTransfer();
        if (sam_loc != ToboRuckus.MineralDetection.SampleLocation.CENTER)
            chassis.rotateTo(0.4, -90);

        chassis.driveStraightAuto(.4, 5, 0, 1000);
        chassis.driveStraightAuto(0.4, 90, -70, 3000);
        Thread.sleep(100);
    }

    @Deprecated
    @MenuEntry(label = "Test Land", group = "Test Auto")
    public void landAndDetach(EventManager em, boolean skipLanding) throws InterruptedException {
        // chassis.resetOrientation();
        if ((hanging != null) && !skipLanding) {
            chassis.driveStraightAuto(0.1, 0.1, 90, 100);
            hanging.latchUpInches(7.75);
            if (!Thread.currentThread().isInterrupted())
                Thread.sleep(30);
        }
        chassis.driveStraightAuto(0.25, -4, 0, 500); //Drive back ~2 in.
        chassis.driveStraightAuto(0.25, 10, -90, 1000); //Strafe left ~4 in.
        chassis.driveStraightAuto(0.25, 4, 0, 500); //Drive forward ~2 in.

        chassis.rotateTo(0.4, -90); //Turn 90 degrees left
//        if (!Thread.currentThread().isInterrupted())
//            Thread.sleep(200);
//        chassis.rotateTo(0.18, -90);

        //for testing
//        Thread.sleep(10000);
//        chassis.driveStraightAuto(0.25, 2, 90, 3000); //Strafe right ~1 in.
//        chassis.driveStraightAuto(0.25, -4, 0, 3000); //Drive back ~2 in.
    }

    public void hopefullyTheLastLandAndDetachForState(EventManager em, boolean skipLanding) throws InterruptedException {
        // chassis.resetOrientation();
        if ((hanging != null) && !skipLanding) {
            chassis.driveStraightAuto(0.1, 0.1, 90, 1000);
            hanging.latchUpInches(7.75);
//            if (!Thread.currentThread().isInterrupted())
//                Thread.sleep(100);
        }
        chassis.driveStraightAuto(0.3, -5, 0, 500); //Drive back ~2 in.
        chassis.driveStraightAuto(0.3, 10, -90, 1000); //Strafe left ~4 in.
        chassis.driveStraightAuto(0.3, 5, 0, 500); //Drive forward ~2 in.
    }

    @MenuEntry(label = "Retract Latch", group = "Test Auto")
    public void retractLatch(EventManager em) throws InterruptedException {
        hanging.latchDownInches(8.5);
    }

    @MenuEntry(label = "Test IMU landing", group = "Test Auto")
    public void landIMU(EventManager em) throws InterruptedException {
        chassis.driveStraightAuto(0.1, 0.1, 90, 1000);
        hanging.landWithIMU();
    }

    @MenuEntry(label = "Test SweepSample", group = "Test Auto")
    public void testSweepSample(EventManager em) throws InterruptedException {
        sweepSample();
    }

    public void sweepSample() throws InterruptedException {
        intake.sweeperOut(0.5);
        Thread.sleep(100);
//        intake.sweeperIn(0.5);
//        Thread.sleep(100);
        intake.stopSweeper();
    }

    @MenuEntry(label = "Auto Transfer", group = "Test Auto")
    public void testAutoTransfer(EventManager em) throws InterruptedException {
        autoTransfer();
    }

    public void autoDelivery(MineralDetection.SampleLocation sam_loc, Side side) throws InterruptedException {
        //arm ready to dump
        mineralDelivery.deliveryCombo(intake);
        while (!TaskManager.isComplete("deliveryCombo")) {
            TaskManager.processTasks();
        }
        //        robot.mineralDelivery.gateOpen();
        if (side == Side.GOLD) {
            Thread.sleep(400);//stop mineral momentum
        } else {
            chassis.driveStraightAuto(0.3, -15, +45, 1000);
        }
        mineralDelivery.gateDumpAuto();
        Thread.sleep(500);//for mineral to drop
        mineralDelivery.returnCombo();
        while (!TaskManager.isComplete("returnCombo")) {
            TaskManager.processTasks();
        }
        intake.moveSliderAuto(intake.getSliderContracted() + 50, 0.99, 600);
        intake.moveGate(true);
        if (side == Side.SILVER) {
            chassis.driveStraightAuto(0.3, 15, +45, 1000);
        }
    }

    public boolean autoTransfer() throws InterruptedException {
        // mineralDelivery.armCollectPos();
        mineralDelivery.gateOpen();
        intake.moveBox(true, true);
        mineralDelivery.wristReadyToCollect();
        Thread.sleep(100);
        // intake.mineralDumpCombo();
        // intake.stopSlider();
        intake.moveSliderAuto(intake.getSliderContracted() + 50, 1.0, 1000);
        long iniTime = System.currentTimeMillis();
        while (System.currentTimeMillis()-iniTime<300) { // time out 0.3 sec
            if (intake.proxDetectMineral()) {
                intake.moveGate(true);
                return true;
            }
        }
        return false;
    }

    @MenuEntry(label = "Auto Collect", group = "Test Auto")
    public void testAutoCollect(EventManager em) throws InterruptedException {
        autoCollect(20, false);
    }

    @MenuEntry(label = "Auto CollectLeft", group = "Test Auto")
    public void testAutoCollectLeft(EventManager em) throws InterruptedException {
        autoCollect(28, false);
    }

    public void autoCollect(double dist, boolean duringParking) throws InterruptedException {
        long iniTime = System.currentTimeMillis();
        if (duringParking) {
            intake.moveSliderAuto(intake.getSliderMinSweep() + 400, 1.0, 800);
        } else {
            intake.moveSliderAuto(intake.getSliderMinSweep() - 50, 1.0, 500);
        }
        intake.moveBox(false, false);
        Thread.sleep(200);
        intake.moveGate(false); // close gate
        int tar_pos = (int) (intake.getSliderMinSweep() + (dist - 6) * intake.slider_count_per_inch);
        intake.moveSliderAuto(tar_pos, 1.0, 300);
        tar_pos += 200;
        if (duringParking) intake.sweeperInAuto();
        else intake.sweeperIn();
        intake.moveSliderAuto(tar_pos, 0.7, 500);
        // intake.setSliderPower(orig_pw);
        if (!Thread.currentThread().isInterrupted())
            Thread.sleep(200);
        if (duringParking){
            intake.sweeperOutAuto();
            Thread.sleep(200);
        }
        intake.stopSweeper();
        intake.stopSlider();
    }

    @MenuEntry(label = "Extend Intake Park", group = "Test Auto")
    public void testExtendIntakeForParking(EventManager em) throws InterruptedException {
        extendInakeForParking(750);
    }

    public void extendInakeForParking(long msec) {
//        final String taskName = "extendIntakeParking";
//        if (!TaskManager.isComplete(taskName)) return;
        intake.setSliderAutoPark();
        try {
            Thread.sleep(msec);
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            intake.stopSlider();
        }
//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                intake.setSliderAutoPark();
//                return new Progress() {
//                    @Override
//                    public boolean isDone() {
//                        return (intake.getSliderCurrent() > (intake.getSliderAutoPark() - 10));
//                    }
//                };
//            }
//        }, taskName);
        intake.stopSlider();
    }

    public void initTeleOp() throws InterruptedException {
        // slider out at dump pos
//        intake.moveSlider(intake.getSliderInitOut());
//        if (!Thread.currentThread().isInterrupted())
//            Thread.sleep(500);

        // box gate open
        intake.moveGate(false);

        // dumper gate open
        mineralDelivery.gateOpen();

        // dumper at down position
        mineralDelivery.armDown();
    }

    /**
     * Returns angle (-180 to 180 degrees) between positive Y axis
     * and a line drawn from center of coordinates to (x, y).
     * Negative values are to the left of Y axis, positive are to the right
     */
    private double toDegrees(double x, double y) {
        if (x == 0) return y >= 0 ? 0 : 180;
        return Math.atan2(x, y) / Math.PI * 180;
    }

    /**
     * Returns power adjustment based on left bumper / left trigger state
     * Normal mode = 60% power
     * Left bumper down = slow mode (30% power)
     * Left trigger down = turbo mode (up to 100%) power
     */
    private double powerAdjustment(EventManager source) {
        double adjustment = chassis.getDefaultScale();  // default adjustment
        double trig_num = 0.0;
        if (source.isPressed(Button.RIGHT_BUMPER)) {
            // slow mode uses 30% of power
            adjustment = 0.25;
        } else if ((trig_num = source.getTrigger(Events.Side.RIGHT)) > 0.2) {
            // 0.2 is the dead zone threshold
            // turbo mode uses (100% + 1/2 trigger value) of power
            adjustment = 0.9 + 0.1 * trig_num * trig_num;
        }
        return adjustment;
    }

    public static class MineralDetection extends OpenCVPipeline {
        static Logger<MineralDetection> logger = new Logger<>();
        CameraSystem cameraSystem;

        static {
            logger.configureLogging("Mineral_Detection", Log.VERBOSE);
        }

        /**
         * @param cameraSystem
         * @author Mason Mann
         * Used for sampling during autonomous
         * Rover Ruckus 2018-19
         */
        public MineralDetection(CameraSystem cameraSystem) {
            this.cameraSystem = cameraSystem;
        }

        @Override
        public Mat processFrame(Mat rgba, Mat grayscale) {
//            boolean showContours = false;
//            Mat silverHSV = new Mat();
//            Mat silverThresholded = new Mat();
//            Mat goldHSV = new Mat();
//            Mat goldThresholded = new Mat();
//            List<MatOfPoint> silverContours;
//            List<MatOfPoint> goldContours;
//
//            Imgproc.cvtColor(rgba, silverHSV, Imgproc.COLOR_RGB2HSV, 3);
//            Imgproc.cvtColor(rgba, goldHSV, Imgproc.COLOR_RGB2HSV, 3);
//            Core.inRange(silverHSV, new Scalar(0, 0, 90), new Scalar(0, 0, 100), silverThresholded);
//            Core.inRange(goldHSV, new Scalar(27, 223, 69.8), new Scalar(51, 160, 100), goldThresholded);
//
//            Imgproc.blur(silverThresholded, silverThresholded, new Size(3, 3));
//
//            Imgproc.blur(goldThresholded, goldThresholded, new Size(3, 3));
//            silverContours = new ArrayList<>();
//            goldContours = new ArrayList<>();
//            Imgproc.findContours(silverThresholded, silverContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//            Imgproc.findContours(goldThresholded, goldContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            if (showContours) {
//                Imgproc.drawContours(rgba, silverContours, -1, new Scalar(144, 255, 255), 2, 8);
//                Imgproc.drawContours(rgba, goldContours, -1, new Scalar(255, 144, 255), 2, 8);
//            }
            return rgba;
        }

        public enum SampleLocation {
            LEFT, CENTER, RIGHT, UNKNOWN
        }

        public Mat getMatFromCamera() {
            Mat mat = new Mat();
            cameraSystem.initVuforia();
            Bitmap bitmap = cameraSystem.captureVuforiaBitmap();
            Utils.bitmapToMat(bitmap, mat);
            return mat;
        }

        /**
         * Determines location of gold sample using OpenCV
         *
         * @return SampleLocation Left, Right, Center, or Unknown
         */
        public SampleLocation getGoldPositionCV() {
            Mat mat = getMatFromCamera();
            Mat goldHSV = new Mat();
            Mat silverHSV = new Mat();
            Mat silverThresholded = new Mat();
            Mat goldThresholded = new Mat();
            List<MatOfPoint> silverContours;
            List<MatOfPoint> goldContours;


            // Changes mat color format from RGB565 to HSV
            Imgproc.cvtColor(mat, goldHSV, Imgproc.COLOR_RGB2HSV, 3);
            Imgproc.cvtColor(mat, silverHSV, Imgproc.COLOR_RGB2HSV, 3);
            logger.verbose("Variable goldHSV size: %s", goldHSV.size());
            logger.verbose("Variable silverHSV size: %s", silverHSV.size());

            // Thresholds color to become binary image of sample and background
            Core.inRange(silverHSV, new Scalar(0, 0, 90), new Scalar(16, 15, 100), silverThresholded);
            Core.inRange(goldHSV, new Scalar(29, 163, 177), new Scalar(51, 223, 255), goldThresholded);
            logger.verbose("Thresholded Gold mat size: %s", goldThresholded.size());
            logger.verbose("Thresholded Silver mat size: %s", silverThresholded.size());
            Imgproc.blur(silverThresholded, silverThresholded, new Size(20, 20));
            Imgproc.blur(goldThresholded, goldThresholded, new Size(20, 20));

            // Places images into contour variables to find the mass centers of the objects
            silverContours = new ArrayList<>();
            goldContours = new ArrayList<>();
            Imgproc.findContours(silverThresholded, silverContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(goldThresholded, goldContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            logger.verbose("Gold Contours Size: %s", goldContours.size());
            logger.verbose("Silver Contours Size: %s", silverContours.size());


            MatOfPoint2f goldCurve = new MatOfPoint2f();
            List<Rect> goldRects = new ArrayList<>();
            for (int i = 0; i < goldContours.size(); i++) {
                MatOfPoint2f contour2f = new MatOfPoint2f(goldContours.get(i).toArray());
                double distance = Imgproc.arcLength(contour2f, true) * 0.02;
                Imgproc.approxPolyDP(contour2f, goldCurve, distance, true);

                MatOfPoint points = new MatOfPoint(goldCurve.toArray());

                goldRects.add(Imgproc.boundingRect(points));
            }
            MatOfPoint2f silverCurve = new MatOfPoint2f();
            List<Rect> silverRects = new ArrayList<>();
            for (int i = 0; i < silverContours.size(); i++) {
                MatOfPoint2f contour2f = new MatOfPoint2f(silverContours.get(i).toArray());
                double distance = Imgproc.arcLength(contour2f, true) * 0.02;
                Imgproc.approxPolyDP(contour2f, silverCurve, distance, true);

                MatOfPoint points = new MatOfPoint(silverCurve.toArray());

                silverRects.add(Imgproc.boundingRect(points));
            }

//            List<Moments> silverMu = new ArrayList<>();
//            Point[] silverCoordCenter = new Point[silverMu.size()];
//            for (int i = 0; i < silverContours.size(); i++) {
//                silverMu.add(Imgproc.moments(silverContours.get(i)));
//                logger.verbose("Silver Moments Size: %s Index: %s", silverMu.size(), i);
//                silverCoordCenter[i] = new Point((int) (silverMu.get(i).m10 / silverMu.get(i).m00), (int) (silverMu.get(i).m01 / silverMu.get(i).m00));
//            }
//            List<Moments> goldMu = new ArrayList<>(goldContours.size());
//            Point[] goldCoordCenter = new Point[goldMu.size()];
//            for (int i = 0; i < goldContours.size(); i++) {
//                goldMu.add(Imgproc.moments(goldContours.get(i)));
//                logger.verbose("Gold Moments Size: %s Index: %s", goldMu.size(), i);
//                goldCoordCenter[i] = new Point((int) (goldMu.get(i).m10 / goldMu.get(i).m00), (int) (goldMu.get(i).m01 / goldMu.get(i).m00));
//            }

            boolean isLeft = true;
            boolean isRight = true;

//            for (int i = 0; i < silverCoordCenter.length; i++){
//                if(goldCoordCenter[0].x > silverCoordCenter[i].x) {
//                    isLeft = false;
//                }
//            }
//            for (int i = 0; i < silverCoordCenter.length; i++){
//                if(goldCoordCenter[0].x < silverCoordCenter[i].x) {
//                    isRight = false;
//                }
//            }

            for (int i = 0; i < silverRects.size(); i++) {
                logger.verbose("Silver Rectangle X: %s", silverRects.get(i).x);
                for (int j = 0; j < goldRects.size(); j++) {
                    logger.verbose("Gold Rectangle X: %s", goldRects.get(j).x);
                    if (goldRects.get(j).x > silverRects.get(i).x)
                        logger.verbose("Gold is not on left");
                    isLeft = false;
                    break;
                }
            }
            for (int i = 0; i < silverRects.size(); i++) {
                for (int j = 0; j < goldRects.size(); j++) {
                    if (goldRects.get(j).x < silverRects.get(i).x)
                        logger.verbose("Gold is not on right");
                    isRight = false;
                    break;
                }
            }

            if (isLeft && !isRight)
                return SampleLocation.LEFT;

            if (!isLeft && isRight)
                return SampleLocation.RIGHT;

            if (!isLeft && !isRight)
                return SampleLocation.CENTER;

            return SampleLocation.UNKNOWN;
        }

        private TFObjectDetector tfObjectDetector;
        private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
        private static final String LABEL_GOLD = "Gold Mineral";
        private static final String LABEL_SILVER = "Silver Mineral";

        public void initTensorFlow(HardwareMap hardwareMap, VuforiaLocalizer vuforia) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfObjectDetector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfObjectDetector.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        }

        /**
         * Determines location of gold sample using CameraMineralDetector Lite ML
         * Assumes only 2 leftmost minerals are visible to camera
         *
         * @return SampleLocation Left, Right, Center, or Unknown
         */
        public SampleLocation getGoldPositionTF() {
            tfObjectDetector.activate();
            List<Recognition> updatedRecognitions = tfObjectDetector.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int goldXCoord = -1;
                int silverXCoord = -1;
                for (Recognition recognition :
                        updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD)) {
                        goldXCoord = (int) recognition.getLeft();
                    } else if (recognition.getLabel().equals(LABEL_SILVER)) {
                        silverXCoord = (int) recognition.getLeft();
                    }
                }
                if (goldXCoord < silverXCoord) {
                    return SampleLocation.LEFT;
                } else if (goldXCoord > silverXCoord) {
                    return SampleLocation.CENTER;
                } else if (goldXCoord == -1 && silverXCoord != -1) {
                    return SampleLocation.RIGHT;
                } else return SampleLocation.UNKNOWN;
            }
            tfObjectDetector.shutdown();
            return SampleLocation.UNKNOWN;
        }

    }
}
