package org.firstinspires.ftc.teamcode.hardware.Dummy;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.components.CameraSystem;
import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.diagnostics.MenuEntry;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
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

public class ToboDummy extends Logger<ToboDummy> implements Robot {
    private Telemetry telemetry;
    public CameraMineralDetector cameraMineralDetector;


    public CoreSystem core;
    public ElapsedTime runtime = new ElapsedTime();
    public double rotateRatio = 0.7; // slow down ratio for rotation
    public double motor_count = 0;

    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    @Override
    public void configure(Configuration configuration, Telemetry telemetry, boolean auto) {
        runtime.reset();
        double ini_time = runtime.seconds();
        this.telemetry = telemetry;

//        cameraSystem = new CameraSystem(null);
//        cameraSystem.init(configuration.getHardwareMap());

        this.core = new CoreSystem();
        if (auto) {
            cameraMineralDetector = new CameraMineralDetector().configureLogging("CameraMineralDetector", logLevel);
            cameraMineralDetector.configure(configuration, true);
        }
        info("ToboDummy configure() after init cameraMineralDetector (run time = %.2f sec)", (runtime.seconds() - ini_time));
    }

    @Override
    public void reset(boolean auto) {

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

        em.updateTelemetry(telemetry, 100);
//        if (!hanging.latchIsBusy()) {
//            hanging.resetLatch();
//        }

        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {

            }
        }, Events.Axis.BOTH, Events.Side.RIGHT);

        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.BOTH)) < 0.2 &&
                        Math.abs(currentX) > 0.1) {
                    // left stick with idle right stick rotates robot in place
                    // chassis.rotate(currentX * Math.abs(currentX) * powerAdjustment(source)*rotateRatio);
                } else if (source.getTrigger(Events.Side.RIGHT) < 0.2 && Math.abs(currentX) > 0.1) {
                    // right stick with left stick operates robot in "car" mode
                    double heading = currentX * 90;
                    double power = source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY);
                    debug("sticksOnly(): left / steer, pwr: %.2f, head: %.2f", power, heading);
                    // chassis.driveAndSteer(power * powerAdjustment(source), heading, false);
                }
                else {
                    // chassis.stop();
                }
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);


        // em: [RB] + [Y] for mineral dump combo (move slider to dump, intake box up, open box gate)
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {

            }
        }, Button.Y);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) {
                if (source.isPressed(Button.BACK)) { // default scale back to 0.5
                    // chassis.setDefaultScale(0.7);
                }
            }
        }, Button.A);

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

    public static class MineralDetection extends OpenCVPipeline {
        static Logger<ToboDummy.MineralDetection> logger = new Logger<>();
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
        public ToboDummy.MineralDetection.SampleLocation getGoldPositionCV() {
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
                return ToboDummy.MineralDetection.SampleLocation.LEFT;

            if (!isLeft && isRight)
                return ToboDummy.MineralDetection.SampleLocation.RIGHT;

            if (!isLeft && !isRight)
                return ToboDummy.MineralDetection.SampleLocation.CENTER;

            return ToboDummy.MineralDetection.SampleLocation.UNKNOWN;
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
        public ToboDummy.MineralDetection.SampleLocation getGoldPositionTF() {
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
                    return ToboDummy.MineralDetection.SampleLocation.LEFT;
                } else if (goldXCoord > silverXCoord) {
                    return ToboDummy.MineralDetection.SampleLocation.CENTER;
                } else if (goldXCoord == -1 && silverXCoord != -1) {
                    return ToboDummy.MineralDetection.SampleLocation.RIGHT;
                } else return ToboDummy.MineralDetection.SampleLocation.UNKNOWN;
            }
            tfObjectDetector.shutdown();
            return ToboDummy.MineralDetection.SampleLocation.UNKNOWN;
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