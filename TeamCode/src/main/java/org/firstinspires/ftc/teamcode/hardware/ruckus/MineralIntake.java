package org.firstinspires.ftc.teamcode.hardware.ruckus;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.components.Operation;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * MineralIntake consists of 2 motors (1 controlling the sweeper, and 1 controlling the slider)
 * and 1 servo controlling the angle of the collection box.<br />
 * Expected hardware configuration is:<br />
 * sweeper (motor), slider (motor), box (servo)
 */
public class MineralIntake extends Logger<MineralIntake> implements Configurable {

    public enum BoxPosition { INITIAL, GOLD_COLLECTION, SILVER_COLLECTION, DUMP };

    private DcMotor sweeperMotor;
    private DcMotor sliderMotor;
    private AdjustableServo boxServo;
    private boolean adjustmentMode = false;

    // adjustable servo positions
    private double boxInitial = 120;
    private double boxGoldCollection = 20;
    private double boxSilverCollection = 0;
    private double boxDump = 240;

    // sweeper intake / push out power values
    private double sweeperInPower = 0.4;
    private double sweeperOutPower = -0.1;
    private int sweeperHalfRotationCount = 275; // TBD

    // slider encoder positions
    private double sliderContracted = 0; // contracted
    private double sliderExtended = 7777; // fully extended
    private double sliderSafe = 3500; // minimally extended position that allows for box rotation
    private double sliderDump = 3000; // allows box to rest on back bracket and dump
    private double sliderPower = 0.5; // TBD

    @Override
    public String getUniqueName() {
        return "intake";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        this.adjustmentMode = on;
    }

    @Adjustable(min = 0, max = 250, step = 1)
    public double getBoxInitial() {
        return boxInitial;
    }
    public void setBoxInitial(double position) {
        this.boxInitial = position;
        if (adjustmentMode) this.boxServo.setPosition(position);
    }

    @Adjustable(min = 0, max = 250, step = 1)
    public double getBoxGoldCollection() {
        return boxGoldCollection;
    }
    public void setBoxGoldCollection(double position) {
        this.boxGoldCollection = position;
        if (adjustmentMode) this.boxServo.setPosition(position);
    }

    @Adjustable(min = 0, max = 250, step = 1)
    public double getBoxSilverCollection() {
        return boxSilverCollection;
    }
    public void setBoxSilverCollection(double position) {
        this.boxSilverCollection = position;
        if (adjustmentMode) this.boxServo.setPosition(position);
    }

    @Adjustable(min = 0, max = 250, step = 1)
    public double getBoxDump() {
        return boxDump;
    }
    public void setBoxDump(double position) {
        this.boxDump = position;
        if (adjustmentMode) this.boxServo.setPosition(position);
    }

    @Adjustable(min = 0.0, max = 1.0, step = 0.01)
    public double getSweeperInPower() {
        return sweeperInPower;
    }
    public void setSweeperInPower(double power) {
        this.sweeperInPower = power;
        if (adjustmentMode) this.sweeperMotor.setPower(power);
    }

    @Adjustable(min = -1.0, max = 0.0, step = 0.01)
    public double getSweeperOutPower() {
        return sweeperOutPower;
    }
    public void setSweeperOutPower(double power) {
        this.sweeperOutPower = power;
        if (adjustmentMode) this.sweeperMotor.setPower(power);
    }

    public int getSweeperHalfRotationCount() {
        return sweeperHalfRotationCount;
    }
    public void setSweeperHalfRotationCount(int count) {
        this.sweeperHalfRotationCount = count;
        //FIXME: how to adjust rotation count?
    }

    @Adjustable(min = 0.0, max = 8000.0, step = 1.0)
    public double getSliderExtended() {
        return sliderExtended;
    }
    public void setSliderExtended(double sliderExtended) {
        this.sliderExtended = sliderExtended;
    }

    @Adjustable(min = 0.0, max = 8000.0, step = 1.0)
    public double getSliderSafe() {
        return sliderSafe;
    }
    public void setSliderSafe(double sliderSafe) {
        this.sliderSafe = sliderSafe;
    }

    @Adjustable(min = 0.0, max = 8000.0, step = 1.0)
    public double getSliderDump() {
        return sliderDump;
    }
    public void setSliderDump(double sliderDump) {
        this.sliderDump = sliderDump;
    }

    @Adjustable(min = 0.0, max = 1.0, step = 0.01)
    public double getSliderPower() {
        return sliderPower;
    }
    public void setSliderPower(double sliderPower) {
        this.sliderPower = sliderPower;
    }

    public void configure(Configuration configuration) {
        boxServo = new AdjustableServo(0, 250).configureLogging(
                logTag + ":servo" , logLevel
        );
        boxServo.configure(configuration.getHardwareMap(), "box");
        configuration.register(boxServo);

        sweeperMotor = configuration.getHardwareMap().tryGet(DcMotor.class, "sweeper");
        sweeperMotor.setDirection(DcMotor.Direction.FORWARD);

        sliderMotor = configuration.getHardwareMap().tryGet(DcMotor.class, "sw_slider");
        sliderMotor.setDirection(DcMotor.Direction.FORWARD);

        configuration.register(this);
    }

    public void reset() {
        boxServo.setPosition(this.boxInitial);
        resetMotor(sweeperMotor);
        resetMotor(sliderMotor);
    }

    /**
     * Rotates the collection box to the specified position
     * @param position {@link BoxPosition}
     * @return operation showing whether rotation is complete
     */
    public Operation setBoxPosition(BoxPosition position) {
        double targetPosition;
        switch(position) {
            case INITIAL: targetPosition = boxInitial; break;
            case SILVER_COLLECTION: targetPosition = boxSilverCollection; break;
            case GOLD_COLLECTION: targetPosition = boxGoldCollection; break;
            case DUMP: targetPosition = boxDump; break;
            default: throw new IllegalArgumentException("Unknown box position: " + position);
        }
        double adjustment = Math.abs(boxServo.getPosition() - targetPosition);
        final long doneBy = System.currentTimeMillis() + Math.round(2 * adjustment);
        boxServo.setPosition(targetPosition);
        return new Operation() {
            @Override
            public boolean isFinished() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    private void resetMotor(DcMotor motor) {
        motor.setPower(0.0d);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


}
