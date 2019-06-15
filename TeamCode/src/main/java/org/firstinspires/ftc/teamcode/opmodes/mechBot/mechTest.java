package org.firstinspires.ftc.teamcode.opmodes.mechBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.MechChassis;

/**
 * Created by 28761 on 6/14/2019.
 */

@Autonomous(name = "TEST-MECH", group = "Ruckus")
public class mechTest  extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        MechChassis mc=new MechChassis(hardwareMap);

        telemetry.addData("Robot is Initialized", "ready for start");
        telemetry.update();

        waitForStart();

        while(true){
            if(gamepad1.dpad_up){
                mc.yMove(+1,0.5);
            }else if(gamepad1.dpad_down){
                mc.yMove(-1,0.5);
            }else if(gamepad1.dpad_right){
                mc.xMove(+1,0.7);
            }else if(gamepad1.dpad_left){
                mc.xMove(-1,0.7);
            }else{
                mc.yMove(0,0);
            }
        }
    }
}
