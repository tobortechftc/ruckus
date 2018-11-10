package org.firstinspires.ftc.teamcode.hardware.ruckus;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

/**
 * Swerve chassis consists of 4 wheels with a Servo and DcMotor attached to each.
 * Track (distance between left and right wheels), wheelbase (distance between front and back wheels)
 * and wheel radius are adjustable.
 * Expected hardware configuration is:<br />
 * Servos: servoFrontLeft, servoFrontRight, servoBackLeft, servoBackRight.<br />
 * Motors: motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight.<br />
 * Orientation sensors (optional): imu, imu2
 */
public class CameraMineralDetector extends Logger<CameraMineralDetector> implements Configurable {
    static Logger<CameraMineralDetector> logger = new Logger<>();

    static {
        logger.configureLogging("CameraMineralDetector", Log.VERBOSE);
    }

    private static final String VUFORIA_KEY = "AaaZDWL/////AAAAGYIaD+Gn/UUDhEiR/gcOJxdEJlKEpCOKSLPfhYJfYthUNZ0vnEGm0VGPutkNgRq8bq1ufm3eAySnLhkJQ7d4w6VDT7os5FGPEOGPfsIWMYNAFMdX+wlJo2JCyljeSxQtXUd/YileyfYKBXOl2uFA4KnStCC9WkYTUBrAof3H7RGKorzYixDeOpbmCsf25rayjtAUQrKCwG4j6P5rRdxy7SC//v4VC6NirNwgJ/xn1r02/jbx8vUDrDODGyut9iLk06IzMnrq/P01yKOp48clTw0WIKNmVT7WUQweXy+E1w6xwFplTlPkjC+gzerDOpxHPqYg8RusWD2Y/IMlmnk1yzJba1B9Xf9Ih6BJbm/fVwL4";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void setAdjustmentMode(boolean on) {
        // CameraMineralDetector doesn't need an adjustment mode
        // Method is only declared for completeness of subsystem
    }

    @Override
    public String getUniqueName() {
        return "CameraMineralDetector";
    }

    public void configure(Configuration configuration) {
        logger.verbose("Start Configuration");
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

        int tfodMonitorViewId = configuration.getHardwareMap().appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", configuration.getHardwareMap().appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        logger.verbose("CameraMineralDetector status: %s", tfod);


        // register CameraMineralDetector as a configurable component
        configuration.register(this);
    }



    public ToboRuckus.MineralDetection.SampleLocation getGoldPositionTF() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        logger.verbose("Start getGoldPositionTF()");
        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            logger.verbose("Start tfod Activation");
            tfod.activate();
            logger.verbose("tfod activate: ", tfod);
        }

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.startTime();

        while (elapsedTime.seconds() < 5){
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
//                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                logger.verbose("Sample Location: Left");
                                return ToboRuckus.MineralDetection.SampleLocation.LEFT;
//                                    telemetry.addData("Gold Mineral Position", "Left");
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                logger.verbose("Sample Location: Right");
                                return ToboRuckus.MineralDetection.SampleLocation.RIGHT;
//                                    telemetry.addData("Gold Mineral Position", "Right");
                            } else {
                                logger.verbose("Sample Location: Center");
                                return ToboRuckus.MineralDetection.SampleLocation.CENTER;
//                                    telemetry.addData("Gold Mineral Position", "Center");
                            }
                        }
                    }
//                        telemetry.update();
                }
            }
        }
        if (tfod != null) {
            tfod.deactivate();
            tfod.shutdown();
            logger.verbose("Tfod shutdown", tfod);
        }
        logger.verbose("Sample Location: Unknown");
        return ToboRuckus.MineralDetection.SampleLocation.UNKNOWN;
    }

    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
//    public void setupTelemetry(Telemetry telemetry) {
//        Telemetry.Line line = telemetry.addLine();
//        if (updatedRecognitions.size() > 0)
//            line.addData("CameraMineralDetector", "Recog. Count= %d", new Func<Integer>() {
//                @Override
//                public Integer value() {
//                    return updatedRecognitions.size();
//                }
//            });
//    }
}
