package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.tfod.TFObjectDetectorImpl;
import org.firstinspires.ftc.teamcode.components.CameraSystem;
import org.firstinspires.ftc.teamcode.hardware.rover.Core;
import org.firstinspires.ftc.teamcode.hardware.ruckus.ToboRuckus;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
@Disabled
@Autonomous(name = "Ruckus :: Sample Recognition Testing", group = "Ruckus")
public class MasonSampleRecognitionTesting extends LinearOpMode {
    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        ToboRuckus robot = new ToboRuckus().configureLogging("ToboRuckus", LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);

        try {
            // configure robot and reset all hardware
            robot.configure(configuration, telemetry);
            configuration.apply();
//            robot.reset();



            telemetry.addData("Robot is ready", "Press Play");
            telemetry.update();
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetStartTime();

        // run until driver presses STOP or runtime exceeds 30 seconds
        if (opModeIsActive() && getRuntime() < 30) {
            try {
                ToboRuckus.MineralDetection mineralDetection = new ToboRuckus.MineralDetection(robot.cameraSystem);

//                mineralDetection.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
//                mineralDetection.enable();
//                List<MatOfPoint> silverContours = mineralDetection.getSilverContours();
//                List<MatOfPoint> goldContours = mineralDetection.getGoldContours();
//                for (int i = 0; i < silverContours.size(); i++) {
//                    Rect boundingRect = Imgproc.boundingRect(silverContours.get(i));
//                    telemetry.addData("Silver contour" + Integer.toString(i),
//                            String.format(Locale.getDefault(), "(%d, %d)", (boundingRect.x + boundingRect.width) / 2, (boundingRect.y + boundingRect.height) / 2));
//                }
//                for (int i = 0; i < goldContours.size(); i++) {
//                    Rect boundingRect = Imgproc.boundingRect(goldContours.get(i));
//                    telemetry.addData("Gold contour" + Integer.toString(i),
//                            String.format(Locale.getDefault(), "(%d, %d)", (boundingRect.x + boundingRect.width) / 2, (boundingRect.y + boundingRect.height) / 2));
//                }

            } catch (Exception E) {
                telemetry.addData("Error", E.getMessage());
                handleException(E);
                Thread.sleep(5000);
            }
        }
    }

    protected void handleException(Throwable T) {
        log.error(T.getMessage(), T);
        int linesToShow = 5;
        for(StackTraceElement line : T.getStackTrace()) {
            telemetry.log().add("%s.%s():%d", line.getClassName(), line.getMethodName(), line.getLineNumber());
            if (--linesToShow == 0) break;
        }
        telemetry.update();
    }
}
