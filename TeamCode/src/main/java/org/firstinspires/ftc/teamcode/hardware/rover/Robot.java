package org.firstinspires.ftc.teamcode.hardware.rover;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Core core;
    public Chassis chassis;

    public Robot() {
        core = new Core();
        chassis = new Chassis(core);

    }

    public void init(HardwareMap hwMap) {
        chassis.init(hwMap);
    }
}
