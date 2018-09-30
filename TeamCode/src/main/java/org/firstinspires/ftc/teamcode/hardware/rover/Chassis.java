package org.firstinspires.ftc.teamcode.hardware.rover;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.rover.Core;

public class Chassis {

    public DistanceSensor rangeSensor;

    final Core core;

    Chassis(Core c) {
        core = c;
    }

    void init(HardwareMap hwMap) {
        rangeSensor = hwMap.get(DistanceSensor.class, "rangeSensor");
    }

    public void show_telemetry(Telemetry telemetry) {
        telemetry.addData("range", rangeSensor.getDistance(DistanceUnit.CM));
    }
}
