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
@Autonomous(name = "NickAutonomousTest", group = "Ruckus")
public class NickTest extends LinearOpMode {
    protected static int LOG_LEVEL = Log.VERBOSE;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    int targetMineral = 1; // 0=left, 1=center, 2=right. default to center as it is fastest
    int timeout = 10; // timeout time for driveStraightAuto

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

        robot.AutoRoutineTest();

        if (opModeIsActive()) {
            try {
                // Forward a bit
                robot.chassis.driveStraightAuto(.2, 15, 0, timeout);
                sleep(500);

                // Knock off mineral and go back
                if (targetMineral == 0) {
                    robot.chassis.driveStraightAuto(.2, 56.6, -49, timeout);
                    sleep(300);
                    robot.chassis.driveStraightAuto(.2, -37, -55, timeout);
                }
                else if (targetMineral == 1) {
                    robot.chassis.driveStraightAuto(.2, 40.6, 0, timeout);
                    sleep(300);
                    robot.chassis.driveStraightAuto(.2, -40.6, 0, timeout);
                }
                else {
                    robot.chassis.driveStraightAuto(.2, 56.6, 49, timeout);
                    sleep(300);
                    robot.chassis.driveStraightAuto(.2, -42.2, 55, timeout);
                }
                sleep(300);
                robot.chassis.driveStraightAuto(.2, 100, -90, timeout);
                sleep(300);
                robot.chassis.rotateTo(.4, -135, telemetry);
                robot.chassis.driveStraightAuto(.2,8,90, timeout);
                sleep(300);
                robot.chassis.driveStraightAuto(.2, 100, 0, timeout);

            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
                handleException(e);
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
