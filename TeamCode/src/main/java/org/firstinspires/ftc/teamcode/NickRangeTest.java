package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.rover.Robot;
import org.firstinspires.ftc.teamcode.support.YieldHandler;

@Autonomous(name = "NickRangeTest", group = "test")
public class NickRangeTest extends LinearOpMode implements YieldHandler {

    Robot robot;

    double distance = 0;
    double sum = 0;
    int counter = 1;
    double mean = 0;
    double max = 0;
    double min = 1000;
    int outliers = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();

        robot.core.set_yield_handler(this);
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            distance = robot.chassis.rangeSensor.getDistance(DistanceUnit.CM);
            sum += distance;
            mean = sum / counter;
            if (distance > mean + 10 || distance < mean - 10)
                outliers++;
            if (distance > max)
                max = distance;
            if (distance < min)
                min = distance;
            counter++;
            robot.core.yield();
            //sleep(20);



        }

    }

    @Override
    public void on_yield() {
        robot.chassis.show_telemetry(telemetry);
        telemetry.addData("mean:", mean);
        telemetry.addData("counter:", counter);
        telemetry.addData("outliers:", outliers);
        telemetry.addData("max:", max);
        telemetry.addData("min:", min);
        telemetry.addData("time:", getRuntime());
        telemetry.update();
    }
}
