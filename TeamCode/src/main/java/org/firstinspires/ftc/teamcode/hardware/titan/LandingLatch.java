package org.firstinspires.ftc.teamcode.hardware.titan;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

/**
 * Created by carlw on 12/28/2018.
 */

public class LandingLatch extends Logger<LandingLatch> implements Configurable{

    final private CoreSystem core;

    private DcMotor latch;
    private Servo marker;
    private double latch_power = .95;
    private final double MARKER_UP = 0.42;
    private final double MARKER_DOWN = 0.05;
    private final int MAX_LATCH_POS = 10400; //Max distance is 8.75 inches
    private final int LATCH_ENDGAME_POS = 6646; // position for end game to latch
    private final int MIN_LATCH_POS = 0;
    private final int LATCH_COUNT_PER_INCH = 1198;
    private boolean markerIsDown = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public String getUniqueName(){return "LandingLatch";}

    /**
     * Landing latch constructor
     */
    public LandingLatch(CoreSystem core) {
        this.core = core;
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    public void reset(boolean Auto) {
        latch.setPower(0);
        if (marker!=null)
            markerUp();
        if (Auto && (latch!=null)) {
            latch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            latch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void configure(Configuration configuration, boolean auto) {
        // set up motors / sensors as wheel assemblies
        latch = configuration.getHardwareMap().dcMotor.get("latch");
        latch.setPower(0);
        latch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // latch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        latch.setDirection(DcMotorSimple.Direction.REVERSE);
        latch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (auto) {
            marker = configuration.getHardwareMap().servo.get("sv_marker");
            markerUp();
        }
        // register hanging as configurable component
        configuration.register(this);
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
    public void latchUp(boolean force){ // encoder going up
        latch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int cur_pos = latch.getCurrentPosition();
        if (cur_pos>=MAX_LATCH_POS && force==false) {
            latch.setPower(0);
        } else {
            latch.setPower(latch_power);
        }
    }
    public void latchDown(boolean force){ // encoder going down
        latch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int cur_pos = latch.getCurrentPosition();
        if ((cur_pos<=MIN_LATCH_POS) && force==false) {
            latch.setPower(0);
        } else {
            latch.setPower(-latch_power);
        }
    }

    public Progress latchAuto(){ // encoder going to end game
        latch.setPower(0);
        latch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        latch.setTargetPosition(LATCH_ENDGAME_POS);
        int cur_pos = latch.getCurrentPosition();
        if (Math.abs(cur_pos-LATCH_ENDGAME_POS)<20) {
            latch.setPower(0);
            latch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            latch.setPower(latch_power);
        }
        return new Progress() {
            @Override
            public boolean isDone() {
                return !latch.isBusy() || (Math.abs(latch.getCurrentPosition()-LATCH_ENDGAME_POS)<20);
            }
        };
    }

    public void latchDownEndGame() {
        final String taskName = "latchDownEndGame";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return latchAuto();
            }
        }, taskName);
    }

    public void latchStop(){
        if (!latch.isBusy())
            latch.setPower(0);
    }

    public void latchUpInches(double inches) { // encoder going down
        runtime.reset();
        int cur_pos = latch.getCurrentPosition();
        int tar_pos = cur_pos + (int)(inches*LATCH_COUNT_PER_INCH);
        if (tar_pos>MAX_LATCH_POS)
            tar_pos = MAX_LATCH_POS;
        latch.setTargetPosition(tar_pos);
        latch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        latch.setPower(Math.abs(latch_power));
        while (latch.isBusy() && (runtime.seconds()<5.0)) {
            cur_pos = latch.getCurrentPosition();
        }
        latchStop();
    }

    public void latchDownInches(double inches) { // encoder going up
        runtime.reset();
        int cur_pos = latch.getCurrentPosition();
        int tar_pos = cur_pos - (int)(inches*LATCH_COUNT_PER_INCH);
        if (tar_pos< MIN_LATCH_POS)
            tar_pos = MIN_LATCH_POS;
        latch.setTargetPosition(tar_pos);
        latch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        latch.setPower(Math.abs(latch_power));
        while (latch.isBusy() && (runtime.seconds()<5.0)) {
            cur_pos = latch.getCurrentPosition();
        }
        latchStop();
    }

    public boolean latchIsBusy() {
        return latch.isBusy();
    }

    public void resetLatch() {
        latch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            line.addData("Latch", "enc=%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return latch.getCurrentPosition();
                }});

        if(marker!=null){
            line.addData("Marker", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return marker.getPosition();
                }});
        }
    }
}