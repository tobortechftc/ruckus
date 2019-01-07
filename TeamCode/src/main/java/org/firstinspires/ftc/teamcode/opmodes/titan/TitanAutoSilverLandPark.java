package org.firstinspires.ftc.teamcode.opmodes.titan;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.SwerveChassis;
import org.firstinspires.ftc.teamcode.hardware.titan.ToboTitan;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.OpModeTerminationException;
import org.firstinspires.ftc.teamcode.support.YieldHandler;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * Created by Nick on 12/30/2018.
 */

@Autonomous(name = "Titan-Silver-Land-Park", group = "Ruckus")
public class TitanAutoSilverLandPark extends LinearOpMode implements YieldHandler {
    protected static int LOG_LEVEL = Log.VERBOSE;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    int timeout = 10000; // timeout time
    double power = .4; // motor power
    ToboTitan.MineralDetection.SampleLocation sam_loc = ToboTitan.MineralDetection.SampleLocation.CENTER;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        ToboTitan robot = new ToboTitan().configureLogging("ToboTitan", LOG_LEVEL);
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

        robot.core.set_yield_handler(this);

        // Step-1: check random sample position
        ToboTitan.MineralDetection.SampleLocation sam_loc = ToboTitan.MineralDetection.SampleLocation.CENTER;
        sam_loc = robot.cameraMineralDetector.getGoldPositionTF(false);

        // Step-2: landing mission
        robot.landAndDetach(null, false);

        // Ste-3: sample mission
        robot.scoreSample(sam_loc);

        //Step-4: align with walls
        robot.chassis.driveStraightAuto(power, -31.6, 0, timeout);

        if (sam_loc == ToboTitan.MineralDetection.SampleLocation.LEFT)
            robot.chassis.driveStraightAuto(power, 45, -90, timeout);
        else if (sam_loc == ToboTitan.MineralDetection.SampleLocation.CENTER)
            robot.chassis.driveStraightAuto(power, 85, -90, timeout);
        else
            robot.chassis.driveStraightAuto(power, 125, -90, timeout);

        robot.chassis.driveStraightAuto(.2, 15, -90, timeout);
        robot.chassis.rotateTo(.2, 135);

        // 6cm away from wall
        double driveDistance = robot.chassis.distanceToRight() - 5;
        robot.chassis.driveStraightAuto(.2, driveDistance, 90, timeout);


        // Step-5: marker mission
        robot.chassis.driveAlongTheWall(power, 100, 5, SwerveChassis.Wall.LEFT, timeout);
        robot.core.yield_for(.2);
//      robot.chassis.rotateTo(.2, -43);
//      driveDistance = robot.chassis.distanceToLeft() - 5;
//      robot.chassis.driveStraightAuto(power, driveDistance, -90, timeout);
//      robot.core.yield_for(.2);
        // drop marker
        robot.landing.markerDown();
        robot.core.yield_for(.5);
        
        // Step-6 park on the rim
        //robot.goParking(ToboTitan.Side.SILVER);
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

    @Override
    public void on_yield() {
        if (!opModeIsActive())
            throw new OpModeTerminationException();
    }
}
