package org.firstinspires.ftc.teamcode.opmodes.ruckus;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.SwerveChassis;
import org.firstinspires.ftc.teamcode.hardware.ruckus.ToboRuckus;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.OpModeTerminationException;
import org.firstinspires.ftc.teamcode.support.YieldHandler;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * Created by 28761 on 10/13/2018.
 */

@Autonomous(name = "Auto-Silver-Land-Park", group = "Ruckus")
public class RuckusAutoSilverLandPark extends LinearOpMode implements YieldHandler {
    protected static int LOG_LEVEL = Log.VERBOSE;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    int timeout = 10000; // timeout time
    double power = .4; // motor power

    double midTime = 0;
    boolean doTime = false;
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
        } catch (Exception e) {
            telemetry.addData("Init Failed", e.getMessage());
            handleException(e);
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetStartTime();
        robot.core.set_yield_handler(this); // uses this class as yield handler 

        // Step-1: check random sample position
        ToboRuckus.MineralDetection.SampleLocation sam_loc = ToboRuckus.MineralDetection.SampleLocation.CENTER;
        sam_loc = robot.cameraMineralDetector.getGoldPositionTF(true);


        // Step-2: landing mission
        robot.landAndDetach(null, false);

        // Step-3: sample mission
        robot.retrieveSample(sam_loc);

        // Step-4: align with walls
        robot.chassis.driveStraightAuto(.3, (sam_loc == ToboRuckus.MineralDetection.SampleLocation.RIGHT ? -22.6 : -24.6), 0, timeout);
        if (sam_loc == ToboRuckus.MineralDetection.SampleLocation.LEFT)
            robot.chassis.driveStraightAuto(power, 45, -90, timeout);
        else if (sam_loc == ToboRuckus.MineralDetection.SampleLocation.RIGHT)
            robot.chassis.driveStraightAuto(power, 125, -90, timeout);
        else
            robot.chassis.driveStraightAuto(power, 85, -90, timeout);

        robot.chassis.driveStraightAuto(.2, 15, -90, timeout);
        robot.chassis.rotateTo(.3, -43);

        // 5cm away from wall
        double driveDistance = robot.chassis.distanceToLeft() - 5;
        robot.chassis.driveStraightAuto(.2, driveDistance, -90, timeout);

        midTime = getRuntime();
        doTime = true;


        // Step-5: marker mission
        // to depot
        robot.chassis.driveAlongTheWall(power, (sam_loc == ToboRuckus.MineralDetection.SampleLocation.RIGHT ? -83 : -73),
                5, SwerveChassis.Wall.LEFT, timeout);

        // drop marker
        robot.hanging.markerDown();
        robot.core.yield_for(.5);


        // Step-6: park on the rim
        robot.goParking(ToboRuckus.Side.SILVER);
        robot.chassis.driveStraightAuto(.4, 10, 5, timeout);


        // Only for testing
        robot.core.yield_for(1);
        robot.hanging.latchDownInches(7.75);

    }

    protected void handleException(Throwable t) {
        log.error(t.getMessage(), t);
        int linesToShow = 5;
        for (StackTraceElement line : t.getStackTrace()) {
            telemetry.log().add("%s.%s():%d", line.getClassName(), line.getMethodName(), line.getLineNumber());
            if (--linesToShow == 0) break;
        }
        telemetry.update();
    }

    @Override
    public void on_yield() {
        telemetry.addData("Time", getRuntime());
        if (doTime) telemetry.addData("MidTime", getRuntime() - midTime);
        if (!opModeIsActive())
            throw new OpModeTerminationException();
    }
}
