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
 * Created by Nick on 2/28/2019.
 */

@Autonomous(name = "Auto-Silver-Short", group = "Ruckus")
public class RuckusAutoSilverShort extends LinearOpMode implements YieldHandler {
    protected static int LOG_LEVEL = Log.VERBOSE;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    int timeout = 10000; // timeout time
    double power = .4; // motor power

    ToboRuckus.MineralDetection.SampleLocation sam_loc = ToboRuckus.MineralDetection.SampleLocation.CENTER;


    @Override
    public void runOpMode() throws InterruptedException {
        ToboRuckus robot = null;


        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        robot = new ToboRuckus().configureLogging("ToboRuckus", LOG_LEVEL);
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
        if (!opModeIsActive()) {
            return;
        }

        resetStartTime();
        robot.core.set_yield_handler(this); // uses this class as yield handler

        // Step-1: check random sample position
        ToboRuckus.MineralDetection.SampleLocation sam_loc = ToboRuckus.MineralDetection.SampleLocation.CENTER;
        sam_loc = robot.cameraMineralDetector.getGoldPositionTF(true);


        // Step-2: landing mission
        robot.landAndDetach(null, false);

        // Step-3: sample mission
        robot.retrieveSample(sam_loc);

        // Step-4: parking mission
        robot.chassis.driveStraightAuto(power, 5, 0, timeout);
        robot.extendInakeForParking();

        // Only for testing
//        robot.core.yield_for(1);
//        robot.hanging.latchDownInches(7.75);

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
        if (!opModeIsActive())
            throw new OpModeTerminationException();
    }
}
