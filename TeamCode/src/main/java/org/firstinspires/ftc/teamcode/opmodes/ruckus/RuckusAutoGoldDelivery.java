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
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

/**
 * Created by 28761 on 3/15/2019.
 */
@Autonomous(name = "Gold-Delivery", group = "Ruckus")
public class RuckusAutoGoldDelivery extends LinearOpMode implements YieldHandler {
    protected static int LOG_LEVEL = Log.VERBOSE;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    double toCm = 2.54;

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
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
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
        robot.hopefullyTheLastLandAndDetachForState(null, false);

        //step-3: retrieve sample
        switch (sam_loc) {//Gold:Silver
            case CENTER: // center
                robot.chassis.rotateTo(0.4, -90);
                robot.autoCollect(20,false);
                break;
            case RIGHT:
                robot.chassis.rotateTo(0.4, -55);
                robot.autoCollect(28,false);
                break;
            case LEFT:
                robot.chassis.rotateTo(0.4, -125);
                robot.autoCollect(28,false);
                break;
            default: // go straight like center
                robot.chassis.rotateTo(0.4, -90);
                robot.autoCollect(20,false);
        }
        boolean gotMineral = robot.autoTransfer();
        if (sam_loc != ToboRuckus.MineralDetection.SampleLocation.CENTER) {
            robot.chassis.rotateTo(0.4, -90);
        } else {
            Thread.sleep(500);//for mineral to drop
        }
        if (gotMineral) {
            robot.autoDelivery(sam_loc, ToboRuckus.Side.GOLD);
        }
        //crab towards the wall
        robot.chassis.driveStraightAuto(0.4, 5, 0, 1000);
        robot.chassis.driveStraightAuto(0.5, 88, -71, 3000);//power was 0.4
        Thread.sleep(100);

        //step-4: deliver marker
//        robot.chassis.driveStraightAuto(0.4, 90, -64, 3000);
        robot.chassis.rotateTo(0.45, +135);//power was 0.4
        Thread.sleep(200);
        robot.chassis.driveStraightAuto(0.2, robot.chassis.getDistance(SwerveChassis.Direction.RIGHT) - 6, +90, 500);
        if (sam_loc == ToboRuckus.MineralDetection.SampleLocation.RIGHT)
            robot.chassis.driveStraightAuto(0.6, -70, 0, 3000);//power was 0.4
        else if (sam_loc == ToboRuckus.MineralDetection.SampleLocation.CENTER)
            robot.chassis.driveStraightAuto(0.6, -75, 0, 3000);//power was 0.4
        else
            robot.chassis.driveStraightAuto(0.6, -80, 0, 3000);//power was 0.4
        robot.hanging.markerDown();
        Thread.sleep(100);
        //step-5: go parking
        robot.chassis.driveStraightAuto(0.5, 20, 5, 400);//power was 0.3
        robot.goParking(ToboRuckus.Side.GOLD, true);

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
