package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * Created by 28761 on 6/14/2019.
 */

public class MechChassis {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    public MechChassis(HardwareMap hardwareMap) {
        motorFL = hardwareMap.tryGet(DcMotor.class, "motorFL");
        motorFR = hardwareMap.tryGet(DcMotor.class, "motorFR");
        motorBL = hardwareMap.tryGet(DcMotor.class, "motorBL");
        motorBR = hardwareMap.tryGet(DcMotor.class, "motorBR");
    }

    public void yMove(int sgn) {
        motorFL.setPower(sgn*0.5);
        motorFR.setPower(-sgn*0.5);
        motorBL.setPower(sgn*0.5);
        motorBR.setPower(-sgn*0.5);
    }

    public void xMove(int sgn) {
        motorFL.setPower(sgn*0.5);
        motorFR.setPower(sgn*0.5);
        motorBL.setPower(-sgn*0.5);
        motorBR.setPower(-sgn*0.5);
    }

    public void turn(int sgn) {
        motorFL.setPower(sgn*0.5);
        motorFR.setPower(sgn*0.5);
        motorBL.setPower(sgn*0.5);
        motorBR.setPower(sgn*0.5);
    }

    public void carDrive(int sgn) {
        motorFL.setPower(sgn*0.5);
        motorFR.setPower(sgn*0.5);
        motorBL.setPower(sgn*0.5);
        motorBR.setPower(sgn*0.5);
    }
}
