package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.rover.Robot;
import org.firstinspires.ftc.teamcode.support.YieldHandler;

@Autonomous(name = "NickRangeTest", group = "test")
public class NickRangeTest extends LinearOpMode implements YieldHandler {

    Robot robot;

    double currentDistance = 0;
    double previousDistance = 0;
    int outliers = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();

        robot.core.set_yield_handler(this);
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            previousDistance = currentDistance;
            currentDistance = robot.chassis.rangeSensor.getDistance(DistanceUnit.CM);

            if (currentDistance > previousDistance + 10)
                outliers++;

            robot.core.yield();
        }

    }

    @Override
    public void on_yield() {
        robot.chassis.show_telemetry(telemetry);
        telemetry.addData("outliers:", outliers);
        telemetry.addData("time:", getRuntime());
        telemetry.update();
    }
}
