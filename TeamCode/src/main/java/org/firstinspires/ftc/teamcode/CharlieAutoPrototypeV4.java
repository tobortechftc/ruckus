package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.minibot.Robot;
import org.firstinspires.ftc.teamcode.support.YieldHandler;

/**
 * Created by 28761 on 9/15/2018.
 */

@Autonomous(name = "MiniBot: CharlieAutoTest V4", group = "MiniBot")
public class CharlieAutoPrototypeV4 extends LinearOpMode implements YieldHandler {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();
        robot.core.set_yield_handler(this);
        robot.init(hardwareMap);
        waitForStart();
        timeStart = System.currentTimeMillis();
        robot.chassis.drive_time_without_imu(0.4, 6000);
    }

    long timeStart;
    int statge;

    long lapsed_time() {
        return System.currentTimeMillis() - timeStart;
    }

    @Override
    public void on_yield() {
        telemetry.addLine("on_yield at time"+lapsed_time());
        if (lapsed_time() > 1000) {
            if (statge == 0) {
                statge++;
                robot.pusher.pusher_up();
            }
        }
        if (lapsed_time() > 2000) {
            if (statge == 1) {
                statge++;
                robot.kicker.kicker_up();
            }
        }
        if (lapsed_time() > 3000) {
            if (statge == 2) {
                statge++;
                robot.pusher.pusher_down();
            }
        }
        if (lapsed_time() > 4000) {
            if (statge == 3) {
                statge++;
                robot.kicker.kicker_down();
            }
        }
        telemetry.update();
    }
}
