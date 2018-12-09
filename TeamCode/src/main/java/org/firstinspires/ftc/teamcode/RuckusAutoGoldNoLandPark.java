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
@Disabled
@Autonomous(name = "Ruckus::Auto-Gold-No-Land-Park", group = "Ruckus")
public class RuckusAutoGoldNoLandPark extends LinearOpMode {
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
        if (opModeIsActive()) {
            resetStartTime();
        }

        // Step-1: check random sample position
        ToboRuckus.MineralDetection.SampleLocation sam_loc = ToboRuckus.MineralDetection.SampleLocation.CENTER;
        if (opModeIsActive()) {
            sam_loc = robot.cameraMineralDetector.getGoldPositionTF(false);
        }

        // Step-2: landing mission
        if (opModeIsActive()) {
            robot.landAndDetach(null, true);
        }
        // Ste-3: sample mission
        if (opModeIsActive()) {
            robot.retrieveSample(sam_loc);
        }
        //Step-4: align with walls
        if (opModeIsActive()) {
            robot.alignWithWallsGoldSide(sam_loc);
        }
        // Step-5: from sample mission to dumping marker
        if (opModeIsActive()) {
            robot.hanging.markerDown();
            robot.chassis.driveStraightAuto(0.3, 20, 10, 3000);
            if (!Thread.currentThread().isInterrupted())
                sleep(500);
        }
        // Step-6: parking on the crater rim
        if (opModeIsActive()) {
            robot.goParking(ToboRuckus.Side.GOLD);
        }

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
