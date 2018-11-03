package org.firstinspires.ftc.teamcode.hardware.ruckus;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SwerveUtilLOP;
import org.firstinspires.ftc.teamcode.components.CameraSystem;
import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.components.SwerveChassis;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
import org.firstinspires.ftc.teamcode.support.diagnostics.MenuEntry;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

public class ToboRuckus extends Logger<ToboRuckus> implements Robot {
    private Telemetry telemetry;
    public SwerveChassis chassis;
    public MineralIntake intake;
    public MineralDelivery mineralDelivery;
    public Hanging hanging;
    public CameraSystem cameraSystem;



    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    @Override
    public void configure(Configuration configuration, Telemetry telemetry) {
        this.telemetry = telemetry;
        
        cameraSystem = new CameraSystem(null);
        cameraSystem.init(configuration.getHardwareMap());
        chassis = new SwerveChassis().configureLogging("Swerve", logLevel);
        chassis.configure(configuration);
        intake = new MineralIntake().configureLogging("Intake", logLevel);
        intake.configure(configuration);
        hanging = new Hanging().configureLogging("Hanging", logLevel);
        hanging.configure(configuration);
        mineralDelivery = new MineralDelivery().configureLogging("Delivery", logLevel);
        mineralDelivery.configure(configuration);
    }

    public void AutoRoutineTest() throws InterruptedException {
        chassis.driveAndSteerAuto(0.6, 560*3, 45);
    }

    @Override
    public void reset() {
        chassis.reset();
        intake.reset();
    }

    @MenuEntry(label = "TeleOp", group = "Competition")
    public void mainTeleOp(EventManager em, EventManager em2) {
        telemetry.addLine().addData("(RS)", "4WD").setRetained(true)
                .addData("(RS) + (LS)", "2WD / Steer").setRetained(true);
        telemetry.addLine().addData("< (LS) >", "Rotate").setRetained(true)
                .addData("[LB]/[LT]", "Slow / Fast").setRetained(true);
        chassis.setupTelemetry(telemetry);
        intake.setupTelemetry(telemetry);
        hanging.setupTelemetry(telemetry);
        mineralDelivery.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        if (!hanging.latchIsBusy()) {
            hanging.resetLatch();
        }
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (source.getStick(Events.Side.LEFT, Events.Axis.BOTH) == 0) {
                    // right stick with idle left stick operates robot in "crab" mode
                    double power = Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.BOTH));
                    double heading = toDegrees(currentX, currentY);
                    // invert headings less than -90 / more than 90
                    if (Math.abs(heading) > 90) {
                        heading -= Math.signum(heading) * 180;
                        power = -1 * power;
                    }
                    debug("sticksOnly(): straight, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, true);
                } else {
                    // right stick with left stick operates robot in "car" mode
                    double heading = source.getStick(Events.Side.LEFT, Events.Axis.X_ONLY) * 90;
                    double power = currentY;
                    debug("sticksOnly(): right / steer, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, false);
                }
            }
        }, Events.Axis.BOTH, Events.Side.RIGHT);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (source.getStick(Events.Side.RIGHT, Events.Axis.BOTH) == 0) {
                    // left stick with idle right stick rotates robot in place
                    chassis.rotate(currentX * powerAdjustment(source));
                } else {
                    // right stick with left stick operates robot in "car" mode
                    double heading = currentX * 90;
                    double power = source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY);
                    debug("sticksOnly(): left / steer, pwr: %.2f, head: %.2f", power, heading);
                    chassis.driveAndSteer(power * powerAdjustment(source), heading, false);
                }
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);
        em.onTrigger(new Events.Listener() {
            @Override
            public void triggerMoved(EventManager source, Events.Side side, float current, float change) throws InterruptedException {
                // 0.2 is a dead zone threshold for the trigger
                if (current > 0.2) {
                    intake.rotateSweeper( MineralIntake.SweeperMode.PUSH_OUT);
                } else if (current == 0) {
                    intake.rotateSweeper( MineralIntake.SweeperMode.HORIZONTAL_STOP);
                }
            }
        }, Events.Side.LEFT);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                intake.rotateSweeper( MineralIntake.SweeperMode.INTAKE);
            }
        }, Button.LEFT_BUMPER);
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                intake.rotateSweeper( MineralIntake.SweeperMode.VERTICAL_STOP);
            }
        }, Button.LEFT_BUMPER);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (button==Button.DPAD_RIGHT) {
                    if (intake.getSliderTarget()==intake.getSliderContracted() || intake.getSliderTarget()==intake.getSliderDump()) {
                        // slider is currently contracted or is in dump position or is moving there
                        intake.moveSlider(intake.getSliderSafe());
                    } else if (intake.getSliderCurrent() >= intake.getSliderSafe()) {
                        intake.moveSlider(intake.getSliderExtended());
                    }
                } else {
                    if (intake.getSliderTarget()==intake.getSliderSafe()) {
                        intake.moveSlider(intake.getSliderDump());
                    } else if (intake.getSliderTarget()==intake.getSliderDump()) {
                        intake.moveSlider(intake.getSliderContracted());
                    } else if (intake.getSliderCurrent() >= intake.getSliderSafe()) {
                        intake.moveSlider(intake.getSliderSafe());
                    }
                }
            }
        }, Button.DPAD_LEFT, Button.DPAD_RIGHT);
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                if (intake.getSliderCurrent() >= intake.getSliderSafe()) intake.stopSlider();
            }
        }, Button.DPAD_LEFT, Button.DPAD_RIGHT);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (button==Button.DPAD_UP) {
                    if (intake.getBoxPosition()==MineralIntake.BoxPosition.GOLD_COLLECTION
                            || intake.getBoxPosition()==MineralIntake.BoxPosition.SILVER_COLLECTION
                    ) {
                        intake.rotateBox(MineralIntake.BoxPosition.INITIAL);
                    } else if (intake.getBoxPosition()==MineralIntake.BoxPosition.INITIAL) {
                        intake.rotateBox(MineralIntake.BoxPosition.DUMP);
                    }
                } else {
                    if (intake.getBoxPosition()==MineralIntake.BoxPosition.GOLD_COLLECTION) {
                        intake.rotateBox(MineralIntake.BoxPosition.SILVER_COLLECTION);
                    } else if (intake.getBoxPosition()==MineralIntake.BoxPosition.SILVER_COLLECTION) {
                        intake.rotateBox(MineralIntake.BoxPosition.GOLD_COLLECTION);
                    } else if (intake.getBoxPosition()==MineralIntake.BoxPosition.INITIAL) {
                        intake.rotateBox(MineralIntake.BoxPosition.GOLD_COLLECTION);
                    } else if (intake.getBoxPosition()==MineralIntake.BoxPosition.DUMP) {
                        intake.rotateBox(MineralIntake.BoxPosition.INITIAL);
                    }
                }
            }
        }, Button.DPAD_UP, Button.DPAD_DOWN);

        // Events for gamepad2
        em2.onTrigger(new Events.Listener() {
            @Override
            public void triggerMoved(EventManager source, Events.Side side, float current, float change) throws InterruptedException {
                // do not rotate if the robot is currently moving
                if (side==Events.Side.LEFT) { // lift down
                    if (current>0.2) {
                        //mineralDelivery.liftDown();
                    } else {
                        //mineralDelivery.liftStop();
                    }
                }
                if (side==Events.Side.RIGHT) { // latch down
                    if (current>0.2) {
                        if (source.isPressed(Button.START))
                            hanging.latchDownInches(1.0);
                        else
                            hanging.latchDown(source.isPressed(Button.BACK));
                    } else {
                        hanging.latchStop();
                    }
                }
            }
        }, Events.Side.LEFT, Events.Side.RIGHT);
        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (button==Button.LEFT_BUMPER) { // lift up
                    //mineralDelivery.liftUp();
                } else if (button==Button.RIGHT_BUMPER) { // latch up
                    if (source.isPressed(Button.START))
                        hanging.latchUpInches(1.0);
                    else
                        hanging.latchUp();
                }
                if (button==Button.B) {
                    if (source.isPressed(Button.BACK)) {
                        hanging.markerAuto();
                    }
                    else if (source.isPressed(Button.START)==false) {
                        hanging.hookAuto();
                    }
                }
                if (button==Button.X){
                    if (source.isPressed(Button.BACK)) {
                        hanging.markerDown();
                    } else {
                        mineralDelivery.gateAuto();
                    }
                }
                if (button==Button.Y) {
                    mineralDelivery.armUp();
                } else if (button==Button.A){
                    mineralDelivery.armDown();
                }
            }
        }, Button.LEFT_BUMPER,Button.RIGHT_BUMPER,Button.B, Button.X, Button.Y, Button.A);

        em2.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                if (button==Button.LEFT_BUMPER) { // lift stop
                    //mineralDelivery.liftStop();
                } else if (button==Button.RIGHT_BUMPER) { // latch stop
                    hanging.latchStop();
                }
            }
        }, Button.LEFT_BUMPER,Button.RIGHT_BUMPER);

        em2.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY) > 0.2) {
                    mineralDelivery.liftUp();
                } else if (source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY) < -0.2) {
                    mineralDelivery.liftDown(source.isPressed(Button.BACK));
                } else {
                    mineralDelivery.liftStop();
                }
            }
        }, Events.Axis.Y_ONLY, Events.Side.LEFT);
        em2.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY) > 0.2) {
                    mineralDelivery.armUpInc();
                } else if (source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY) < -0.2) {
                    mineralDelivery.armDownInc();
                } else {
                    mineralDelivery.armStop();
                }
            }
        }, Events.Axis.Y_ONLY, Events.Side.RIGHT);
    }

    @MenuEntry(label = "Drive Straight", group = "Test Chassis")
    public void testStraight(EventManager em) {
        telemetry.addLine().addData("(LS)", "Drive").setRetained(true)
                .addData("Hold [LB]/[RB]", "45 degree").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
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

    /**
     * Returns angle (-180 to 180 degrees) between positive Y axis
     *  and a line drawn from center of coordinates to (x, y).
     * Negative values are to the left of Y axis, positive are to the right
     */
    private double toDegrees(double x, double y) {
        if (x == 0) return y>=0 ? 0 : 180;
        return Math.atan2(x, y) / Math.PI * 180;
    }

    /**
     * Returns power adjustment based on left bumper / left trigger state
     * Normal mode = 60% power
     * Left bumper down = slow mode (30% power)
     * Left trigger down = turbo mode (up to 100%) power
     */
    private double powerAdjustment(EventManager source) {
        double adjustment = 0.6;
        if (source.isPressed(Button.RIGHT_BUMPER)) {
            // slow mode uses 30% of power
            adjustment = 0.3;
        } else if (source.getTrigger(Events.Side.RIGHT) > 0.2) {
            // 0.2 is the dead zone threshold
            // turbo mode uses (100% + 1/2 trigger value) of power
            adjustment = 0.6 + 0.4 * source.getTrigger(Events.Side.RIGHT);
        }
        return adjustment;
    }
}
