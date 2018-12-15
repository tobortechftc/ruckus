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

@Autonomous(name = "Auto-Silver-Land-Park", group = "Ruckus")
public class RuckusAutoSilverLandPark extends LinearOpMode {
    protected static int LOG_LEVEL = Log.VERBOSE;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    int timeout = 10000; // timeout time
    double power = .4; // motor power
    ToboRuckus.MineralDetection.SampleLocation sam_loc = ToboRuckus.MineralDetection.SampleLocation.CENTER;


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
            sam_loc = robot.cameraMineralDetector.getGoldPositionTF(false);
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
        // go to wall and turn parallel
        if (opModeIsActive()) {
            // go back
            robot.chassis.driveStraightAuto(power, -31.6, 0, timeout);

            if (sam_loc == ToboRuckus.MineralDetection.SampleLocation.LEFT)
                robot.chassis.driveStraightAuto(power, 45, -90, timeout);
            else if (sam_loc == ToboRuckus.MineralDetection.SampleLocation.CENTER)
                robot.chassis.driveStraightAuto(power, 85, -90, timeout);
            else
                robot.chassis.driveStraightAuto(power, 125, -90, timeout);

            robot.chassis.driveStraightAuto(.2, 15, -90, timeout);
            robot.chassis.rotateTo(.2, -43);

            // 6cm away from wall
            double driveDistance = robot.chassis.distanceToLeft() - 5;
            robot.chassis.driveStraightAuto(.2, driveDistance, -90, timeout);
        }

        // stpe-5: marker mission
        if (opModeIsActive()) {
            // to depot
            robot.chassis.driveAlongTheWall(power, -100, 5, SwerveChassis.Wall.LEFT, timeout);
            // realign
            if (opModeIsActive()) sleep(200);
//                robot.chassis.rotateTo(.2, -43);
//                driveDistance = robot.chassis.distanceToLeft() - 5;
//                robot.chassis.driveStraightAuto(power, driveDistance, -90, timeout);
//                sleep(200);

            // drop marker
            robot.hanging.markerDown();
            if (opModeIsActive()) sleep(500);
        }

        // Step-5 park on the rim
        if (opModeIsActive()) {
            robot.goParking(ToboRuckus.Side.SILVER);
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
