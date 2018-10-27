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
    private double gateClosePos = 0;    // minimum power that should be applied to the wheel motors for robot to start moving
    private double gateOpenPos = .11200;    // maximum power that should be applied to the wheel motors
    private double armDownPos = 0;
    private double armUpPos = 0.1;
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
        lift = configuration.getHardwareMap().dcMotor.get("lift");
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dumperGate = configuration.getHardwareMap().servo.get("sv_hp_gate");
        dumperArm = configuration.getHardwareMap().servo.get("sv_hp_dump");
        gateClose();



        // register chassis as configurable component
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
    public void setup_telemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        if (lift!=null)
           line.addData("Latch", "pw=%.2f pos=%.2f", lift.getPower(), lift.getCurrentPosition());

        if(dumperGate!=null){
            line.addData("Hook", "pos=%.2f", dumperGate.getPosition());
        }
        if(dumperArm!=null){
            line.addData("Hook", "pos=%.2f", dumperArm.getPosition());
        }
    }

}
