package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.ruckus.ToboRuckus;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * Created by 28761 on 10/13/2018.
 */

@Autonomous(name = "Auto-Gold-Land", group = "Ruckus")
public class RuckusAutoGoldLand extends LinearOpMode {
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
            sam_loc = robot.cameraMineralDetector.getGoldPositionTF(true);
        }
        // Step-2: landing mission
        if (opModeIsActive()) {
            robot.landAndDetach(null, false);
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
            if (!Thread.currentThread().isInterrupted())
                sleep(500);
        }
        // Step-5: parking on the crater rim
        if (opModeIsActive()) {
            robot.chassis.driveStraightAuto(0.3, 20, 0, 3000);
//            robot.goParkingGold();
//            robot.intake.moveSlider(robot.intake.getSliderInitOut());
        }
//        robot.AutoRoutineTest();
        // run until driver presses STOP or runtime exceeds 30 seconds
        if (opModeIsActive() && getRuntime() < 30) {
            try {
                // TODO: invoke something like robot.autonomousProgram()
                telemetry.addLine(String.format("distance Left:%.3f; distance front:%.3f", robot.chassis.distanceToLeft(), robot.chassis.distanceToFront()));
                telemetry.update();
//                robot.chassis.driveAndSteerAuto(0.5,560*3,-45);

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
        for (StackTraceElement line : T.getStackTrace()) {
            telemetry.log().add("%s.%s():%d", line.getClassName(), line.getMethodName(), line.getLineNumber());
            if (--linesToShow == 0) break;
        }
        telemetry.update();
    }
}
