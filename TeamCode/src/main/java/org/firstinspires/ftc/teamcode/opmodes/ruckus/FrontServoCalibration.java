package org.firstinspires.ftc.teamcode.opmodes.ruckus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 28761 on 2/24/2019.
 */

@TeleOp(name="front marker calibration", group="Ruckus")
public class FrontServoCalibration extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo frontMarker = hardwareMap.servo.get("sv_FrontMarker");
        while (true) {
            telemetry.addLine(String.format("current position: %f", frontMarker.getPosition()));
            if (gamepad2.dpad_up) {
                frontMarker.setPosition(Math.min(frontMarker.getPosition() + 0.01, 1));
            } else if (gamepad2.dpad_down) {
                frontMarker.setPosition(Math.min(frontMarker.getPosition() - 0.01, 1));
            } else {
                //do nothing
            }
        }

    }
}
