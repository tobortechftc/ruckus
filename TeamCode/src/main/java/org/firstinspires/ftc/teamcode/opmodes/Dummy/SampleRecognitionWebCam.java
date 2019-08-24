package org.firstinspires.ftc.teamcode.opmodes.Dummy;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Dummy.ToboDummy;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

@Autonomous(name = "Dummy :: Sample Recognition WebCam", group = "Dummy")
public class SampleRecognitionWebCam extends LinearOpMode {
    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        ToboDummy robot = new ToboDummy().configureLogging("ToboDummy", LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);

        try {
            // configure robot and reset all hardware
            robot.configure(configuration, telemetry, true);
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
        ToboDummy.MineralDetection.SampleLocation pos = ToboDummy.MineralDetection.SampleLocation.UNKNOWN;
        // run until driver presses STOP or runtime exceeds 30 seconds
        while (opModeIsActive()) {
            try {
                if (pos==ToboDummy.MineralDetection.SampleLocation.UNKNOWN) {
                    pos = robot.cameraMineralDetector.getGoldPositionTF(false);
                    telemetry.addData("Gold Position = ", pos).setRetained(true);
                    telemetry.update();
                }
                //Thread.sleep(10000);

            } catch (Exception E) {
                telemetry.addData("Error", E.getMessage());
                handleException(E);
                //Thread.sleep(5000);
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
