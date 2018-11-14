package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.ruckus.MineralIntake;
import org.firstinspires.ftc.teamcode.hardware.ruckus.ToboRuckus;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * Created by 28761 on 11/9/2018.
 */

@Autonomous(name = "Ruckus::Auto-Gold-2(New)", group = "Ruckus")
public class RuckusAutoGold2 extends LinearOpMode {
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
            robot.configure(configuration, telemetry,true);
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



//        if (robot.hanging!=null) {
//            robot.chassis.driveStraightAuto(0.1, 0.1, 90, 1000);
//            robot.hanging.latchUpInches(7);//Land
//            sleep(2000);
//        }
//        robot.chassis.driveStraightAuto(0.25, -5, 0, 3000); //Drive back ~2 in.
//        sleep(200);
//        robot.chassis.driveStraightAuto(0.25, 12.5, -90, 3000); //Strafe left ~4 in.
//        sleep(200);
//        robot.chassis.driveStraightAuto(0.25, 5, 0, 3000); //Drive forward ~2 in.
//        sleep(200);
//        robot.chassis.rotateTo(0.25, -80, telemetry); //Turn 90 degrees left

        //at this place, use open cv to determine the mineral configuration

        int mode = -1;
        if (mode == 0) {
            robot.chassis.driveStraightAuto(0.35, 43, 0,Integer.MAX_VALUE);
        } else if (mode == 1) {
            robot.chassis.driveStraightAuto(0.35, 58, 46,Integer.MAX_VALUE);
        } else {
            robot.chassis.driveStraightAuto(0.35, 58, -46,Integer.MAX_VALUE);
        }

//        robot.intake.rotateBox(MineralIntake.BoxPosition.GOLD_COLLECTION);
        robot.intake.rotateSweeper(MineralIntake.SweeperMode.INTAKE);
        sleep(500);
        robot.chassis.driveStraightAuto(0.35, 30, 0,Integer.MAX_VALUE);
        sleep(500);
        robot.intake.stopSweeper();

        robot.chassis.rotateDegree(0.3, -135);
        robot.chassis.rotateTo(0.18, -135);
        

        //from here, three different routine will converge into the depot
        sleep(1000);

        //align with right wall
        double detectedRightDistance=robot.chassis.distanceToRight();
        telemetry.addLine(String.format("detected distance to right: %.3f",detectedRightDistance));
        telemetry.update();
        sleep(1000);
        robot.chassis.driveStraightAuto(0.30, detectedRightDistance-15,+90,Integer.MAX_VALUE);
        telemetry.addLine(String.format("adjusted distance to right: %.3f",robot.chassis.distanceToRight()));
        telemetry.update();

        //force heading correction
        sleep(500);
        robot.chassis.rotateTo(0.18, -135);

        //align with back wall
        telemetry.addLine(String.format("detected distance to back: %.3f",robot.chassis.distanceToBack()));
        telemetry.update();
        sleep(500);
        robot.chassis.driveStraightAuto(0.30, 30.0 - robot.chassis.distanceToBack(),0,Integer.MAX_VALUE);
        telemetry.addLine(String.format("adjusted distance to back: %.3f",robot.chassis.distanceToBack()));
        telemetry.update();


        //dump the marker here
        robot.hanging.markerDown();
        sleep(500);

        telemetry.addLine("MARKER DROP!!!!!");
        telemetry.update();
        sleep(2000);
//        robot.chassis.rotateTo(0.3,-5);
//        sleep(200);
//        robot.chassis.rotateTo(0.3,5);
//        sleep(200);
//        robot.chassis.rotateTo(0.3,0);

//        robot.chassis.rotateTo(0.18, 46);
//
//        sleep(500);
//        robot.chassis.driveStraightAuto(0.40, -180,0,Integer.MAX_VALUE);
//        sleep(200);
//        robot.chassis.driveStraightAuto(0.20, 5,90,Integer.MAX_VALUE);
//
////        robot.chassis.rotateTo(0.18, 60);
//        robot.chassis.driveStraightAuto(0.70, -40,0,2000);

//        robot.chassis.driveAndSteerAuto(0.7, 1500, 45, telemetry);
////        robot.chassis.rotateDegree(0.4,45);
//        sleep(500);
//        robot.chassis.rotateTo(0.5, 45);
//        sleep(500);
//
//        robot.chassis.driveAndSteerAuto(0.5, 1800, -90, telemetry);
//        sleep(500);
//        robot.chassis.rotateTo(0.5, 45);
//
//        robot.chassis.driveAndSteerAuto(-0.5, 560 * 3, 0, telemetry);

//        robot.AutoRoutineTest();
        // run until driver presses STOP or runtime exceeds 30 seconds
        if (opModeIsActive() && getRuntime() < 30) {
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
