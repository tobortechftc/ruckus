package org.firstinspires.ftc.teamcode.opmodes.ruckus;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.ruckus.ToboRuckus;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.OpModeTerminationException;
import org.firstinspires.ftc.teamcode.support.YieldHandler;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * Created by 28761 on 2/24/2019.
 */
@Autonomous(name = "Auto-New-Gold", group = "Ruckus")
public class RuckusHopefullyTheFinalGoldAutonomousForState extends LinearOpMode implements YieldHandler {
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
        robot.core.set_yield_handler(this); // uses this class as yield handler


        // Step-1: check random sample position
        ToboRuckus.MineralDetection.SampleLocation sam_loc = ToboRuckus.MineralDetection.SampleLocation.CENTER;
        sam_loc = robot.cameraMineralDetector.getGoldPositionTF(true);

        // Step-2: landing mission
        robot.hopefullyTheLastLandAndDetachForState(null, false);

        switch (sam_loc) {
            case CENTER: // center
                robot.chassis.rotateTo(0.4, -90);
                break;
            case RIGHT:
                robot.chassis.rotateTo(0.4, -60);
                break;
            case LEFT:
                robot.chassis.rotateTo(0.4, -125);
                break;
            default: // go straight like center
                robot.chassis.rotateTo(0.4, -90);
        }
        if (sam_loc != ToboRuckus.MineralDetection.SampleLocation.CENTER)
            robot.chassis.rotateTo(0.4, -90);
        robot.chassis.driveStraightAuto(0.4, 75, -60, 3000);
        robot.chassis.rotateTo(0.4, -135);

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
