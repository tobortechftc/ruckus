package org.firstinspires.ftc.teamcode.hardware.ruckus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.components.CombinedOrientationSensor;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.util.Arrays;

/**
 * Swerve chassis consists of 4 wheels with a Servo and DcMotor attached to each.
 * Track (distance between left and right wheels), wheelbase (distance between front and back wheels)
 *  and wheel radius are adjustable.
 * Expected hardware configuration is:<br />
 * Servos: servoFrontLeft, servoFrontRight, servoBackLeft, servoBackRight.<br />
 * Motors: motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight.<br />
 * Orientation sensors (optional): imu, imu2
 */
public class Hanging extends Logger<Hanging> implements Configurable {

    private DcMotor latch;
    private Servo hook;
    private Servo marker;
    private double minLatchPos = 0;    // minimum power that should be applied to the wheel motors for robot to start moving
    private double maxLatchPos = 11200;    // maximum power that should be applied to the wheel motors
    private double hook_up = 0.55;
    private double hook_down = 0.05;
    private double latch_power = .5;
    private boolean hookIsOpened = false;
    private final double MARKER_UP = 0.4;
    private final double MARKER_DOWN = 0.9;
    private boolean markerIsDown = false;

    @Override
    public String getUniqueName() {
        return "hanging";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    public void reset() {
        latch.setPower(0);
        hookClose();
        markerUp();
    }

    public void configure(Configuration configuration) {
        // set up motors / sensors as wheel assemblies
        latch = configuration.getHardwareMap().dcMotor.get("latch");
        latch.setPower(0);
        latch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        latch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hook = configuration.getHardwareMap().servo.get("sv_hook");
        hookClose();
        marker = configuration.getHardwareMap().servo.get("sv_marker");
        markerUp();

        // register hanging as configurable component
        configuration.register(this);
    }

    public void hookClose(){
        hook.setPosition(hook_up);
        hookIsOpened=false;
    }
    public void hookOpen(){
        hook.setPosition(hook_down);
        hookIsOpened=true;
    }
    public void hookAuto() {
        if (hookIsOpened) {
            hookClose();
        } else {
            hookOpen();
        }
    }
    public void markerUp(){
        marker.setPosition(MARKER_UP);
        markerIsDown = false;
    }
    public void markerDown(){
        marker.setPosition(MARKER_DOWN);
        markerIsDown = true;
    }
    public void markerAuto(){
        if (markerIsDown) {
            markerUp();
        }
        else {
            markerDown();
        }
    }
    public void latchUp(){
        latch.setPower(latch_power);
    }
    public void latchDown(){
        latch.setPower(-1 * latch_power);
    }
    public void latchStop(){
        latch.setPower(0);
    }
    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     *  drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     *  and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        if (latch!=null)
           line.addData("Latch", "pw=%.2f enc=%d", latch.getPower(), latch.getCurrentPosition());

        if(hook!=null){
            line.addData("Hook", "pos=%.2f", hook.getPosition());
        }

        if(marker!=null){
            line.addData("Marker", "pos=%.2f", marker.getPosition());
        }
    }

}



