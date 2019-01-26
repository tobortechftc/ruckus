package org.firstinspires.ftc.teamcode.opmodes.ruckus;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.SwerveChassis;
import org.firstinspires.ftc.teamcode.hardware.ruckus.ToboRuckus;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * Created by Nick on 1/4/2018.
 */
@Disabled
@Autonomous(name = "Auto-Silver-Land-Double", group = "Ruckus")
public class RuckusAutoSilverLandDouble extends LinearOpMode {
    protected static int LOG_LEVEL = Log.VERBOSE;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    int timeout = 10000; // timeout time
    double power = .4; // motor power
    double driveDistance = 0;


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


        // Step-1: check random sample position
        ToboRuckus.MineralDetection.SampleLocation sam_loc = ToboRuckus.MineralDetection.SampleLocation.CENTER;

        if (opModeIsActive()) {
            sam_loc = robot.cameraMineralDetector.getGoldPositionTF(true);
        }

        // Step-2: landing mission
        if (opModeIsActive()) {
            robot.landAndDetach(null, false);
        }
        // Step-3: sample mission
        if (opModeIsActive()) {
            robot.retrieveSample(sam_loc);
        }

        //Step-4: align with walls
        // go to wall and turn parallel
        if (opModeIsActive()) {
            // go back
            robot.chassis.driveStraightAuto(power, (sam_loc == ToboRuckus.MineralDetection.SampleLocation.RIGHT ? -22.6 : -26.6), 0, timeout);

            if (sam_loc == ToboRuckus.MineralDetection.SampleLocation.LEFT)
                robot.chassis.driveStraightAuto(power, 45, -90, timeout);
            else if (sam_loc == ToboRuckus.MineralDetection.SampleLocation.RIGHT)
                robot.chassis.driveStraightAuto(power, 125, -90, timeout);
            else
                robot.chassis.driveStraightAuto(power, 85, -90, timeout);

            robot.chassis.driveStraightAuto(.2, 15, -90, timeout);
            robot.chassis.rotateTo(.2, -45);

            // 5cm away from wall
            double driveDistance = robot.chassis.distanceToLeft() - 5;
            robot.chassis.driveStraightAuto(.2, driveDistance, -90, timeout);
        }

        // Step-5: marker mission
        if (opModeIsActive()) {
            // to depot
            robot.chassis.driveAlongTheWall(power, -100, 5, SwerveChassis.Wall.LEFT, timeout);
            // realign
            if (opModeIsActive()) sleep(200);
            robot.chassis.rotateTo(.2, -45);
            // drop marker
            sleep(200);
            robot.hanging.markerDown();

            // right=(60,117) center=(89,89) left(117,60) wheel = (38,35)
            double wheelX = 38 + robot.chassis.getDistance(SwerveChassis.Direction.LEFT);
            double wheelY = 35 + robot.chassis.getDistance(SwerveChassis.Direction.BACK);
            double deltaX;
            double deltaY;
            double distance;
            double heading;
            double offset;
            int outcome = 0;
            sleep(100);
            if (sam_loc == ToboRuckus.MineralDetection.SampleLocation.LEFT) {
                deltaX = 117 - wheelX;
                deltaY = 62 - wheelY;
            } else if (sam_loc == ToboRuckus.MineralDetection.SampleLocation.CENTER) {
                deltaX = 89 - wheelX;
                deltaY = 89 - wheelY;
            } else {
                deltaX = 62 - wheelX;
                deltaY = 117 - wheelY;
            }
            distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY) + 3;
            offset = 45 + robot.chassis.orientationSensor.getHeading();
            heading = Math.toDegrees(Math.atan(deltaX / deltaY)) - offset;
            if (heading > 90)
                heading -= 180;
            if (heading < 0) {
                distance *= -1;
                outcome = 1;
            }
            sleep(300);
            robot.chassis.driveStraightAuto(power, distance, heading, timeout);
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
