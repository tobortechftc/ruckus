package org.firstinspires.ftc.teamcode.support;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.ruckus.ToboRuckus;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * Created by 28761 on 10/27/2018.
 */
@Disabled
@Autonomous(name = "Ruckus :: Charlie Crater Clearance", group = "Ruckus")
public class CharlieCraterClear extends LinearOpMode {
    protected static int LOG_LEVEL = Log.VERBOSE;

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

        robot.chassis.driveStraightAuto(0.9,-40,0,2000);
        //at this place, use open cv to determine the mineral configuration


//        robot.AutoRoutineTest();
        // run until driver presses STOP or runtime exceeds 30 seconds
        while (opModeIsActive() && getRuntime() < 30) {
            try {
                // TODO: invoke something like robot.autonomousProgram()
                telemetry.addLine(String.format("distance Left:%.3f; distance front:%.3f",robot.chassis.distanceToLeft(),robot.chassis.distanceToFront()));
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
