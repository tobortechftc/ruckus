package org.firstinspires.ftc.teamcode.hardware.ruckus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * Swerve chassis consists of 4 wheels with a Servo and DcMotor attached to each.
 * Track (distance between left and right wheels), wheelbase (distance between front and back wheels)
 *  and wheel radius are adjustable.
 * Expected hardware configuration is:<br />
 * Servos: servoFrontLeft, servoFrontRight, servoBackLeft, servoBackRight.<br />
 * Motors: motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight.<br />
 * Orientation sensors (optional): imu, imu2
 */
public class MineralDelivery extends Logger<MineralDelivery> implements Configurable {

    private DcMotor lift;
    private Servo dumperArm;
    private Servo dumperGate;
    private double gateClosePos = 0.01;
    private double gateOpenPos = .8;
    private double armDownPos = 0.01;
    private double armUpPos = 0.95;
    private double liftPower = .5;
    private boolean gateIsOpened = false;

    @Override
    public String getUniqueName() {
        return "delivery";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    public void reset() {
        lift.setPower(0);
        gateClose();
    }

    public void configure(Configuration configuration) {
        // set up motors / sensors as wheel assemblies
        lift = configuration.getHardwareMap().dcMotor.get("lift_slider");
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dumperGate = configuration.getHardwareMap().servo.get("sv_hp_gate");
        gateOpen();
        dumperArm = configuration.getHardwareMap().servo.get("sv_hp_dump");
        armDown();

        // register delivery as configurable component
        configuration.register(this);
    }

    public void gateClose(){
        dumperGate.setPosition(gateClosePos);
        gateIsOpened=false;
    }
    public void gateOpen(){
        dumperGate.setPosition(gateOpenPos);
        gateIsOpened=true;
    }
    public void gateAuto() {
            if (gateIsOpened) {
            gateClose();
        } else {
            gateOpen();
        }
    }
    public void liftUp(){
        lift.setPower(liftPower);
    }
    public void liftDown(){
        lift.setPower(-1 * liftPower);
    }
    public void liftStop(){
        lift.setPower(0);
    }
    public void armUp(){
        dumperArm.setPosition(armUpPos);
    }
    public void armDown(){
        dumperArm.setPosition(armDownPos);
    }
    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     *  drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     *  and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        if (lift!=null)
           line.addData("Lift", "pw=%.2f enc=%d", lift.getPower(), lift.getCurrentPosition());

        if(dumperGate!=null){
            line.addData("Gate", "pos=%.2f", dumperGate.getPosition());
        }
        if(dumperArm!=null){
            line.addData("Arm", "pos=%.2f", dumperArm.getPosition());
        }
    }

}
