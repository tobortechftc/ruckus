package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.ruckus.ToboRuckus;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

// Robot is not "prepped" at right time (servos unaligned until start button pressed)
// Robot drove forward an incorrect distance, and then shut itself off
@Autonomous(name = "NickTest", group = "test")
public class NickTest extends LinearOpMode {
    ToboRuckus robot;
    Configuration config;

    double time = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new ToboRuckus();
        config = new Configuration(hardwareMap, "rover");
        robot.configure(config, telemetry);

        waitForStart();

        if (opModeIsActive()) {
            robot.chassis.drive_and_steer_auto(.2, 20, 30);
            robot.chassis.drive_and_steer_auto(.1, 0, -45);
            time = getRuntime();
            robot.chassis.drive_straight(-.2, 0);
            while (time + 5 > getRuntime()) {    }
            robot.chassis.drive_straight(0,0);
        }
    }
}
