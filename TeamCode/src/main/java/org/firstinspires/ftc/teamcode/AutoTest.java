package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.ruckus.ToboRuckus;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * Created by 28761 on 10/13/2018.
 */
@Autonomous(name = "Auto Test", group = "Ruckus")
public class AutoTest extends LinearOpMode {
    protected static int LOG_LEVEL = Log.VERBOSE;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    double toCm = 2.54;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        ToboRuckus robot = new ToboRuckus().configureLogging("ToboRuckus", LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);

        try {
            // configure robot and reset all hardware
            robot.configure(configuration, telemetry, true);
            configuration.apply();
            robot.reset(true);

            telemetry.addData("Robot is ready", "Press Play");
            telemetry.update();
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetStartTime();

//        robot.extendInakeForParking();

        ToboRuckus.MineralDetection.SampleLocation sam_loc= ToboRuckus.MineralDetection.SampleLocation.CENTER;
        if (opModeIsActive()) {
            sam_loc = robot.cameraMineralDetector.getGoldPositionTF();
        }
        telemetry.addData("1. Camera detect gold at", "%s (time=%.2f)", sam_loc.name(), getRuntime());
        telemetry.update();
        robot.chassis.resetOrientation();
        telemetry.addData("2. Reset IMUs", "%s (time=%.2f)", sam_loc.name(), getRuntime());
        telemetry.update();
        sleep(5000);
//         robot.chassis.rotateTo(0.18, -90);
//         sleep(200);
//         robot.chassis.rotateTo(0.18, +180);
//         sleep(200);
//         robot.chassis.rotateTo(0.18, 90);
//         sleep(200);
//         robot.chassis.rotateTo(0.18, 0);
//         sleep(200);

    }

    protected void handleException(Throwable T) {
        log.error(T.getMessage(), T);
        int linesToShow = 5;
        for (StackTraceElement line : T.getStackTrace()) {
            telemetry.log().add("%s.%s():%d", line.getClassName(), line.getMethodName(), line.getLineNumber());
            if (--linesToShow == 0) break;
        }
        telemetry.update();
    }
}
