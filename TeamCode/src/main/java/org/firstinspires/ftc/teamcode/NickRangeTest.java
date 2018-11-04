package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.minibot.Robot;
import org.firstinspires.ftc.teamcode.support.YieldHandler;
@Disabled
@TeleOp(name = "NickRangeTest", group = "test")
public class NickRangeTest extends LinearOpMode implements YieldHandler {

    Robot robot;

    double currentDistance = 0;
    double power = 0;
    boolean isMoving = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();

        robot.core.set_yield_handler(this);
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            currentDistance = robot.chassis.rangeSensor.getDistance(DistanceUnit.CM);
            if (isMoving)
                Log.v("range", String.format("%.2f", currentDistance));

            if (gamepad1.a) {
                setSpeed(0);
                isMoving = false;
            }

            if (gamepad1.dpad_up) {
                setSpeed(power);
                isMoving = true;
            }
            if (gamepad1.dpad_down) {
                setSpeed(-power);
                isMoving = true;
            }

            if (gamepad1.dpad_right)
                power += .1;
            if (gamepad1.dpad_left)
                power -= .1;

            robot.core.yield();
        }

    }

    void setSpeed(double x) {
        robot.chassis.motorLeft.setPower(x);
        robot.chassis.motorRight.setPower(x);
    }
    @Override
    public void on_yield() {
    }
}
