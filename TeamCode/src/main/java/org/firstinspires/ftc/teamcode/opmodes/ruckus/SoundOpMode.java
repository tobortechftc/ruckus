package org.firstinspires.ftc.teamcode.opmodes.ruckus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="SoundOpMode", group="Ruckus")
public class SoundOpMode extends OpMode{

    private boolean pressedLast;
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        this.pressedLast = false;
    }
    public void loop() {
        if (this.gamepad1.x && !this.pressedLast) {
            CenaPlayer.start(this.hardwareMap.appContext, telemetry);
            this.pressedLast = true;
            telemetry.addData("Status", "Playing the song");
            telemetry.update();
        }
        else if (!this.gamepad1.x && this.pressedLast) {
            CenaPlayer.stop();
            this.pressedLast = false;
            telemetry.addData("Status", "Stop the song");
            telemetry.update();
        }
    }
    public void stop() {
        CenaPlayer.stop();
    }

}
