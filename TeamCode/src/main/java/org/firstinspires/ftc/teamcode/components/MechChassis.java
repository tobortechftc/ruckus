package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * move in the vertical direction
     *
     * @param sgn   can be either +1 or -1
     * @param power must be in range [0,1]
     */
    public void yMove(int sgn, double power) {
        motorFL.setPower(sgn * power);
        motorFR.setPower(sgn * power);
        motorBL.setPower(sgn * power);
        motorBR.setPower(sgn * power);
    }

    /**
     * move in the horizontal direction
     *
     * @param sgn   can be either +1 or -1
     * @param power must be in range [0,1]
     */
    public void xMove(int sgn, double power) {
        motorFL.setPower(sgn * power);
        motorFR.setPower(-sgn * power);
        motorBL.setPower(-sgn * power);
        motorBR.setPower(sgn * power);
    }

    /**
     * pivot and turn
     *
     * @param sgn   can be either +1 or -1(+1 for clockwise, -1 for counter clockwise)
     * @param power must be in range [0,1]
     */
    public void turn(int sgn, double power) {
        motorFL.setPower(sgn * power);
        motorFR.setPower(-sgn * power);
        motorBL.setPower(sgn * power);
        motorBR.setPower(-sgn * power);
    }

    /**
     * turning while driving
     *
     * @param power         must be in range [0,1]
     * @param turningFactor int range [-1,+1] (+1 for turning right, -1 for turning left)
     */
    public void carDrive(double power, double turningFactor) {
        if (turningFactor>0){
            turningFactor = 1 - turningFactor;
            motorFL.setPower(power);
            motorFR.setPower(turningFactor * power);
            motorBL.setPower(power);
            motorBR.setPower(turningFactor * power);
        }else {
            turningFactor = 1 + turningFactor;
            motorFL.setPower(turningFactor * power);
            motorFR.setPower(power);
            motorBL.setPower(turningFactor * power);
            motorBR.setPower(power);
        }

    }
}
