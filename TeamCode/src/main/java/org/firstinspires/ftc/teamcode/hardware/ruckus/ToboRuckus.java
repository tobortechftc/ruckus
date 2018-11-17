package org.firstinspires.ftc.teamcode.hardware.ruckus;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.CameraSystem;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.components.SwerveChassis;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
import org.firstinspires.ftc.teamcode.support.diagnostics.MenuEntry;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
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
    public SwerveChassis chassis;
    public MineralIntake intake;
    public MineralDelivery mineralDelivery;
    public Hanging hanging;
    public CameraSystem cameraSystem;
    public CameraMineralDetector cameraMineralDetector;


    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    @Override
    public void configure(Configuration configuration, Telemetry telemetry, boolean auto) {
        this.telemetry = telemetry;

//        cameraSystem = new CameraSystem(null);
//        cameraSystem.init(configuration.getHardwareMap());
        if (auto) {
            cameraMineralDetector = new CameraMineralDetector().configureLogging("CameraMineralDetector", logLevel);
            cameraMineralDetector.configure(configuration);
        }
        chassis = new SwerveChassis().configureLogging("Swerve", logLevel); // Log.DEBUG
        chassis.configure(configuration);
        intake = new MineralIntake().configureLogging("Intake", logLevel);
        intake.configure(configuration);
        hanging = new Hanging().configureLogging("Hanging", logLevel);
        hanging.configure(configuration);
        mineralDelivery = new MineralDelivery().configureLogging("Delivery", logLevel);
        mineralDelivery.configure(configuration);
    }

    public void AutoRoutineTest() throws InterruptedException {
        chassis.driveAndSteerAuto(0.6, 560 * 3, 45);
    }

    @Override
    public void reset(boolean auto) {
        chassis.reset();
        intake.reset();
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
                    intake.rotateSweeper(MineralIntake.SweeperMode.PUSH_OUT);
                } else if (current == 0) {
                    intake.rotateSweeper(MineralIntake.SweeperMode.HORIZONTAL_STOP);
                }
            }
        }, Events.Side.LEFT);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                intake.rotateSweeper(MineralIntake.SweeperMode.INTAKE);
            }
        }, Button.LEFT_BUMPER);
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                intake.rotateSweeper(MineralIntake.SweeperMode.VERTICAL_STOP);
            }
        }, Button.LEFT_BUMPER);

        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (button==Button.DPAD_LEFT) {
                    if (source.isPressed(Button.BACK)) {
                        intake.adjustSlider(true);
                    } else if (intake.getSliderCurrent() >= intake.getSliderDump()) {
                        intake.moveSlider(intake.getSliderExtended());
                    } else {
                        intake.moveSlider(intake.getSliderDump());
                    }
                } else {
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
        em2.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) {
                if (intake.getSliderCurrent() >= intake.getSliderDump()) intake.stopSlider();
            }
        }, Button.DPAD_LEFT, Button.DPAD_RIGHT);
        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.BACK)) {
                    // manually open / close the gate
                    intake.moveGate(button == Button.DPAD_UP);
                } else if (intake.getSliderTarget()>=intake.getSliderDump()) {
                    // if slider is extended, move the box but not the lid
                    intake.moveBox(button == Button.DPAD_UP);
                } else {
                    // if slider is in the dump position, move the box and the lid
                    intake.operateIntake(button == Button.DPAD_DOWN);
                }
            }
        }, Button.DPAD_UP, Button.DPAD_DOWN);

        // Events for gamepad2
        em2.onTrigger(new Events.Listener() {
            @Override
            public void triggerMoved(EventManager source, Events.Side side, float current, float change) {
                // do not rotate if the robot is currently moving
                if (side == Events.Side.LEFT) { // lift down
                    if (current > 0.2) {
                        //mineralDelivery.liftDown();
                    } else {
                        //mineralDelivery.liftStop();
                    }
                }
                if (side == Events.Side.RIGHT) { // latch down
                    if (current > 0.2) {
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
            public void buttonDown(EventManager source, Button button) {
                if (button == Button.LEFT_BUMPER) { // lift up
                    //mineralDelivery.liftUp();
                } else if (button == Button.RIGHT_BUMPER) { // latch up
                    if (source.isPressed(Button.START))
                        hanging.latchUpInches(1.0);
                    else
                        hanging.latchUp(source.isPressed(Button.BACK));
                }
                if (button == Button.B) {
                    if (source.isPressed(Button.BACK)) {
                        hanging.markerAuto();
                    }
                }
                if (button == Button.X) {
                    if (source.isPressed(Button.BACK)) {
                        hanging.markerDown();
                    } else {
                        mineralDelivery.gateAuto();
                    }
                }
                if (button == Button.Y) {
                    mineralDelivery.armDump();
                } else if (button == Button.A) {
                    mineralDelivery.armDown();
                }
            }
        }, Button.LEFT_BUMPER, Button.RIGHT_BUMPER, Button.B, Button.X, Button.Y, Button.A);

        em2.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) {
                if (button == Button.LEFT_BUMPER) { // lift stop
                    //mineralDelivery.liftStop();
                } else if (button == Button.RIGHT_BUMPER) { // latch stop
                    hanging.latchStop();
                }
            }
        }, Button.LEFT_BUMPER, Button.RIGHT_BUMPER);

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
        em2.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) {
                if (source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY) > 0.2) {
                    mineralDelivery.armUpInc();
                } else if (source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY) < -0.2) {
                    mineralDelivery.armDownInc();
                } else {
                    mineralDelivery.armStop();
                }
            }
        }, Events.Axis.Y_ONLY, Events.Side.RIGHT);
        em2.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                mineralDelivery.operateDelivery();
            }
        }, Button.RIGHT_STICK);
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

    @MenuEntry(label = "Test Sample", group = "Test Auto")
    public void goGetSampleGold(ToboRuckus.MineralDetection.SampleLocation sam_loc ) throws InterruptedException{
        switch(sam_loc) {
            case CENTER: // center
                chassis.driveStraightAuto(0.35, 43, 0, Integer.MAX_VALUE);
                break;
            case RIGHT:
                chassis.driveStraightAuto(0.35, 58, 46, Integer.MAX_VALUE);
                break;
            case LEFT:
                chassis.driveStraightAuto(0.35, 58, -46, Integer.MAX_VALUE);
                break;
            default: // go straight like center
                chassis.driveStraightAuto(0.35, 43, 0, Integer.MAX_VALUE);
        }
        intake.rotateSweeper(MineralIntake.SweeperMode.INTAKE);
        Thread.sleep(100);
        chassis.driveStraightAuto(0.35, 10, 0,Integer.MAX_VALUE);
        Thread.sleep(500);
        intake.stopSweeper();
    }

    @MenuEntry(label = "Test Sample", group = "Test Auto")
    public void alignWithWallsGoldSide() throws InterruptedException{
        //drive to default start position for gold side
        chassis.driveStraightAuto(0.35, 20, 0,Integer.MAX_VALUE);
        Thread.sleep(500);
        //rotate robot parallel to the walls
//        telemetry.addLine("imu heading:%");
        chassis.rotateTo(0.3, 135);
        chassis.rotateTo(0.18, 135);

        //from here, three different routine will converge into the depot
        Thread.sleep(100);

        //align with right wall
        double detectedRightDistance=chassis.distanceToRight();
        chassis.driveStraightAuto(0.30, detectedRightDistance-15,+90,Integer.MAX_VALUE);
//        telemetry.addLine(String.format("adjusted distance to right: %.3f",chassis.distanceToRight()));
//        telemetry.update();

        //force heading correction
        Thread.sleep(500);
        chassis.rotateTo(0.18, 135);

        //align with back wall
//        telemetry.addLine(String.format("detected distance to back: %.3f",chassis.distanceToBack()));
//        telemetry.update();
        Thread.sleep(500);
        chassis.driveStraightAuto(0.30, 30.0 - chassis.distanceToBack(),0,Integer.MAX_VALUE);
//        telemetry.addLine(String.format("adjusted distance to back: %.3f",chassis.distanceToBack()));
//        telemetry.update();
    }

    @MenuEntry(label = "Test Land", group = "Test Auto")
    public void landAndDetach(EventManager em, boolean skipLanding) throws InterruptedException{
        chassis.resetOrientation();
        if ((hanging!=null)&&!skipLanding) {
            chassis.driveStraightAuto(0.1, 0.1, 90, 1000);
            hanging.latchUpInches(8);
            Thread.sleep(500);
        }
        chassis.driveStraightAuto(0.25, -5, 0, 3000); //Drive back ~2 in.
        chassis.driveStraightAuto(0.25, 12.5, -90, 3000); //Strafe left ~4 in.
        chassis.driveStraightAuto(0.25, 5, 0, 3000); //Drive forward ~2 in.

        chassis.rotateTo(0.3, -80); //Turn 90 degrees left
        chassis.rotateTo(0.18, -90);

        chassis.driveStraightAuto(0.25, 2, 90, 3000); //Strafe right ~1 in.
        chassis.driveStraightAuto(0.25, -4, 0, 3000); //Drive back ~2 in.
    }

    @MenuEntry(label = "Retract Latch", group = "Test Auto")
    public void retractLatch(EventManager em) throws InterruptedException{
        hanging.latchDownInches(8.5);
    }

    public void initTeleOp() throws InterruptedException {
        // slider out at dump pos
        intake.moveSlider(intake.getSliderInitOut());
        Thread.sleep(500);

        // box gate close
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
                if (goldXCoord < silverXCoord){
                    return SampleLocation.LEFT;
                }
                else if (goldXCoord > silverXCoord) {
                    return SampleLocation.CENTER;
                }
                else if (goldXCoord == -1 && silverXCoord != -1){
                    return SampleLocation.RIGHT;
                }
                else return SampleLocation.UNKNOWN;
            }
            tfObjectDetector.shutdown();
            return SampleLocation.UNKNOWN;
        }

//        public synchronized List<MatOfPoint> getGoldContours() {
//            return goldContours;
//        }
//        public synchronized  List<MatOfPoint> getSilverContours(){
//            return silverContours;
//        }
//        public Point[] findSilver(){
//            List<Moments> mu = new ArrayList<>(silverContours.size());
//            Point[] coordCenter = new Point[mu.size()];
//            for (int i = 0; i < silverContours.size(); i++) {
//                mu.add(Imgproc.moments(silverContours.get(i)));
//                coordCenter[i] = new Point((int)(mu.get(i).m10 / mu.get(i).m00), (int)(mu.get(i).m01 / mu.get(i).m00));
//            }
//            return coordCenter;
//        }
//        public Point[] findGold(){
//            List<Moments> mu = new ArrayList<>(goldContours.size());
//            Point[] coordCenter = new Point[mu.size()];
//            for (int i = 0; i < goldContours.size(); i++) {
//                mu.add(Imgproc.moments(goldContours.get(i)));
//                coordCenter[i] = new Point((int)(mu.get(i).m10 / mu.get(i).m00), (int)(mu.get(i).m01 / mu.get(i).m00));
//            }
//            return coordCenter;
//        }
    }
}
