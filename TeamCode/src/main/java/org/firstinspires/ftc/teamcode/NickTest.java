package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.SwerveChassis;
import org.firstinspires.ftc.teamcode.hardware.ruckus.ToboRuckus;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * Created by 28761 on 10/13/2018.
 */

@Autonomous(name = "NickAutonomousTest", group = "Ruckus")
public class NickTest extends LinearOpMode {
    protected static int LOG_LEVEL = Log.VERBOSE;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    int targetMineral = 1; // 0=left, 1=center, 2=right. default to center
    int timeout = 10000; // timeout time
    double power = .3; // motor power

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

        if (opModeIsActive()) {
            try {
                ToboRuckus.MineralDetection.SampleLocation sam_loc = ToboRuckus.MineralDetection.SampleLocation.CENTER;

                // land, detach, sample
                robot.landAndDetach(null, true);
                robot.retrieveSample(sam_loc);

                // go back
                if (targetMineral == 0)
                    robot.chassis.driveStraightAuto(power, -37, -55, timeout); // change distance
                else if (targetMineral == 1)
                    robot.chassis.driveStraightAuto(power, -31.6, 0, timeout);
                else
                    robot.chassis.driveStraightAuto(power, -37, 55, timeout); // change distance

                // go to wall and turn parallel
                robot.chassis.driveStraightAuto(power, 85, -90, timeout);
                robot.chassis.driveStraightAuto(.2, 15, -90, timeout);
                robot.chassis.rotateTo(.2, -45);

                // 6cm away from wall
                double driveDistance = robot.chassis.distanceToLeft() - 5;
                robot.chassis.driveStraightAuto(.2, driveDistance, -90, timeout);

                // to depot
                robot.chassis.driveStraightAuto(power, -100, 0, timeout);

                // realign
                sleep(200);
                robot.chassis.rotateTo(.2, -45);
                driveDistance = robot.chassis.distanceToLeft() - 5;
                robot.chassis.driveStraightAuto(.2, driveDistance, -90, timeout);
                sleep(200);

                // drop marker
                robot.hanging.markerDown();
                sleep(500);

                // get out of depot
                robot.chassis.driveAlongTheWall(.2, 100, 5, SwerveChassis.Side.LEFT, timeout);


                //robot.chassis.driveStraightAuto(.2, 170, 0, timeout);




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
