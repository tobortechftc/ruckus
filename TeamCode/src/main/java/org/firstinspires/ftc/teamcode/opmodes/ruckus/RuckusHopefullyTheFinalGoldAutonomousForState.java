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
@Autonomous(name = "Gold-Collect", group = "Ruckus")
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

        //step-3: retrieve sample
        switch (sam_loc) {
            case CENTER: // center
                robot.chassis.rotateTo(0.4, -90);
                robot.autoCollect(20);
                break;
            case RIGHT:
                robot.chassis.rotateTo(0.4, -59);
                robot.autoCollect(28);
                break;
            case LEFT:
                robot.chassis.rotateTo(0.4, -125);
                robot.autoCollect(28);
                break;
            default: // go straight like center
                robot.chassis.rotateTo(0.4, -90);
                robot.autoCollect(20);
        }
        robot.autoTransfer();
        if (sam_loc != ToboRuckus.MineralDetection.SampleLocation.CENTER)
            robot.chassis.rotateTo(0.4, -90);

        //step-4: deliver marker
        robot.chassis.driveStraightAuto(0.4, 90, -64, 3000);
        Thread.sleep(200);
        robot.chassis.rotateTo(0.4, +135);
        Thread.sleep(200);
        if (sam_loc == ToboRuckus.MineralDetection.SampleLocation.CENTER)
            robot.chassis.driveStraightAuto(0.4, -75, 0, 3000);
        else
            robot.chassis.driveStraightAuto(0.4, -85, 0, 3000);
        robot.hanging.markerDown();
        Thread.sleep(200);
        //step-5: go parking
        robot.chassis.driveStraightAuto(0.3, 20, 5, 500);
        robot.goParking(ToboRuckus.Side.GOLD);
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
