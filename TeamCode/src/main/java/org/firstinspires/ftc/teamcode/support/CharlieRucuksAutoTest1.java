package org.firstinspires.ftc.teamcode.support;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.ruckus.ToboRuckus;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * Created by 28761 on 10/13/2018.
 */

@Autonomous(name = "Ruckus :: Charlie Autonomous test", group = "Ruckus")
public class CharlieRucuksAutoTest1 extends LinearOpMode {
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
            robot.configure(configuration, telemetry);
            configuration.apply();
            robot.reset();

            telemetry.addData("Robot is ready", "Press Play");
            telemetry.update();
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetStartTime();


        //at this place, use open cv to determine the mineral configuration
        int mode = +1;
        if (mode == 0) {
            robot.chassis.driveStraightAuto(0.35, 43, 0);
        } else if (mode == 1) {
            robot.chassis.driveStraightAuto(0.35, 58, 46);
        } else {
            robot.chassis.driveStraightAuto(0.35, 58, -46);
        }

        sleep(500);
        robot.chassis.driveStraightAuto(0.35, 40, 0);

        sleep(1000);
        robot.chassis.rotateTo(0.5, 45,telemetry);
        sleep(500);
        robot.chassis.driveStraightAuto(0.4, robot.chassis.distanceToLeft() - 10.0,-90);
        sleep(500);
        robot.chassis.driveStraightAuto(0.4, robot.chassis.distanceToFront() - 30.0,0);

        //dump the marker here

        sleep(100);
//        robot.chassis.rotateTo(0.5, 46,telemetry);

        sleep(3000);
        robot.chassis.driveStraightAuto(0.90, 240,0);

//        robot.chassis.driveAndSteerAuto(0.7, 1500, 45, telemetry);
////        robot.chassis.rotateDegree(0.4,45);
//        sleep(500);
//        robot.chassis.rotateTo(0.5, 45,telemetry);
//        sleep(500);
//
//        robot.chassis.driveAndSteerAuto(0.5, 1800, -90, telemetry);
//        sleep(500);
//        robot.chassis.rotateTo(0.5, 45,telemetry);
//
//        robot.chassis.driveAndSteerAuto(-0.5, 560 * 3, 0, telemetry);

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
