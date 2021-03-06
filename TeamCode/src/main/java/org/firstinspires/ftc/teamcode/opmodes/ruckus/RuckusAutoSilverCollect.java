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

@Autonomous(name = "Silver-Collect", group = "Ruckus")
public class RuckusAutoSilverCollect extends LinearOpMode implements YieldHandler {
    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<?> log;

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
        this.log = robot;

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

        if (!opModeIsActive()) {
            return;
        }

        resetStartTime();
        robot.core.set_yield_handler(this); // uses this class as yield handler

        // LOGGER Testing
        robot.info("LOGGING TEST #1 = %s", "this is an INFO message.");
        robot.warn("LOGGING TEST #2 = %s", "this is a WARN message.");
        robot.verbose("LOGGING TEST #3 = %s", "this is a VERBOSE message.");
        robot.error("LOGGING TEST #4 = %s", "this is an ERROR message.");

        // Step-1: check random sample position
        ToboRuckus.MineralDetection.SampleLocation sam_loc = ToboRuckus.MineralDetection.SampleLocation.CENTER;
        logStep("viewing sample positions");
        sam_loc = robot.cameraMineralDetector.getGoldPositionTF(true);


        // Step-2: landing mission
        logStep("landing and detaching");
        robot.hopefullyTheLastLandAndDetachForState(null, false);


        // Step-3: collect and go to wall
        logStep("collecing sample, going to wall");
        robot.collectSampleAndGoToWall(sam_loc, ToboRuckus.Side.SILVER);
        robot.chassis.rotateTo(power, -43);

        // 5cm away from wall
        double driveDistance = robot.chassis.getDistance(SwerveChassis.Direction.LEFT) - 5;
        robot.chassis.driveStraightAuto(.2, driveDistance, -90, 500);

        midTime = getRuntime();
        doTime = true;


        // Step-4: marker mission
        // to depot
        logStep("driving to depot");
        robot.chassis.driveAlongTheWall(power, -104, 5, SwerveChassis.Wall.LEFT, 5000);

        // drop marker
        logStep("dropping marker");
        robot.hanging.markerDown();
        robot.core.yield_for(.5);


        // Step-5: park on the rim
        logStep("driving to crater");
        robot.goParking(ToboRuckus.Side.SILVER,false);
        robot.chassis.driveStraightAuto(.17, 10, 5, 500);


        // Only for testing
//        robot.core.yield_for(1);
//        robot.hanging.latchDownInches(7.75);

    }

    void logStep(String message) {
        log.info("runTime: %s; status: %s", getRuntime(), message);
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
