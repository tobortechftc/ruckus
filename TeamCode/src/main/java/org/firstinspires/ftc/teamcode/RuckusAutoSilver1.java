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

@Autonomous(name = "Ruckus::Auto-Silver-1", group = "Ruckus")
public class RuckusAutoSilver1 extends LinearOpMode {
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
            robot.configure(configuration, telemetry);
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



        if (robot.hanging!=null) {
            robot.chassis.driveStraightAuto(0.1, 0.1, 90, 1000);
            robot.hanging.latchUpInches(7);//Land
            sleep(2000);
        }
        robot.chassis.driveStraightAuto(0.25, -5, 0, 3000); //Drive back ~2 in.
        sleep(200);
        robot.chassis.driveStraightAuto(0.25, 12.5, -90, 3000); //Strafe left ~4 in.
        sleep(200);
        robot.chassis.driveStraightAuto(0.25, 5, 0, 3000); //Drive forward ~2 in.
        sleep(200);
        robot.chassis.rotateTo(0.25, -75, telemetry); //Turn 90 degrees left

        //at this place, use open cv to determine the mineral configuration
        int mode = 0;
        if (mode == 0) {
            robot.chassis.driveStraightAuto(0.35, 43, 0,Integer.MAX_VALUE);
        } else if (mode == 1) {
            robot.chassis.driveStraightAuto(0.35, 58, 46,Integer.MAX_VALUE);
        } else {
            robot.chassis.driveStraightAuto(0.35, 58, -46,Integer.MAX_VALUE);
        }

        sleep(500);
        robot.chassis.driveStraightAuto(0.4, 30, 0,Integer.MAX_VALUE);
        // sleep(1000);
        // robot.chassis.rotateTo(0.25,-40,telemetry);

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
