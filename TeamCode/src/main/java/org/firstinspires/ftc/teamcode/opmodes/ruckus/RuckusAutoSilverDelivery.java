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

@Autonomous(name = "Silver-Delivery", group = "Ruckus")
public class RuckusAutoSilverDelivery extends LinearOpMode implements YieldHandler {
    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    int timeout = 10000; // timeout time
    double power = .4; // motor power

    double midTime = 0;
    boolean doTime = false;
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
        robot.hopefullyTheLastLandAndDetachForState(null, false);


        // Step-3: collect and deliver sample
        switch (sam_loc) {
            case CENTER:
                robot.chassis.rotateTo(0.4, -90);
                robot.autoCollect(20, false);
                break;
            case RIGHT:
                robot.chassis.rotateTo(0.4, -55);
                robot.autoCollect(28, false);
                break;
            case LEFT:
                robot.chassis.rotateTo(0.4, -125);
                robot.autoCollect(28, false);
                break;
            default: // go straight like center
                robot.chassis.rotateTo(0.4, -90);
                robot.autoCollect(20, false);
        }
        boolean gotMineral = robot.autoTransfer();
        if (sam_loc != ToboRuckus.MineralDetection.SampleLocation.CENTER) {
            robot.chassis.rotateTo(0.4, -90); // face center to back up straight
        } else {
            Thread.sleep(500); // wait for mineral to drop
        }
        if (gotMineral) {
            robot.autoDelivery(sam_loc, ToboRuckus.Side.SILVER);
        }
        //crab towards the wall
        robot.chassis.driveStraightAuto(0.4, 5, 0, 1000);
        robot.chassis.driveStraightAuto(0.5, 88, -71, 3000);//power was 0.4
        Thread.sleep(100);
        robot.chassis.rotateTo(0.4, -43);
        Thread.sleep(100);


        // Step-4: marker mission
        // to depot
        robot.chassis.driveAlongTheWall(power, -94, 5, SwerveChassis.Wall.LEFT, 5000);

        // drop marker
        robot.hanging.markerDown();
        robot.core.yield_for(.5);


        // Step-5: park on the rim
        robot.goParking(ToboRuckus.Side.SILVER, true);
        robot.chassis.driveStraightAuto(.17, 10, 5, 500);


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
        telemetry.addData("Time", getRuntime());
        if (doTime) telemetry.addData("MidTime", getRuntime() - midTime);
        if (!opModeIsActive())
            throw new OpModeTerminationException();
    }
}
