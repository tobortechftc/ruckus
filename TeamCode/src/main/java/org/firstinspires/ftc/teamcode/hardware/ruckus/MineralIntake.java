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

    // adjustable servo positions in order of BoxPosition constants
    private double[] boxPositions = { 120, 20, 0, 240 };

    // sweeper intake / push out power values
    private double sweeperInPower = 0.4;
    private double sweeperOutPower = 0.5;
    private int sweeperHalfRotation = 275; // TBD

    // slider encoder positions
    private double sliderContracted = 0; // contracted
    private double sliderExtended = 7777; // fully extended
    // minimally extended position that allows for box to be rotated to one of the collection positions
    private double sliderSafe = 3500;
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
        return this.boxPositions[BoxPosition.INITIAL.ordinal()];
    }
    public void setBoxInitial(double position) {
        this.boxPositions[BoxPosition.INITIAL.ordinal()] = position;
        if (adjustmentMode) this.boxServo.setPosition(position);
    }

    @Adjustable(min = 0, max = 250, step = 1)
    public double getBoxGoldCollection() {
        return this.boxPositions[BoxPosition.GOLD_COLLECTION.ordinal()];
    }
    public void setBoxGoldCollection(double position) {
        this.boxPositions[BoxPosition.GOLD_COLLECTION.ordinal()] = position;
        if (adjustmentMode) this.boxServo.setPosition(position);
    }

    @Adjustable(min = 0, max = 250, step = 1)
    public double getBoxSilverCollection() {
        return this.boxPositions[BoxPosition.SILVER_COLLECTION.ordinal()];
    }
    public void setBoxSilverCollection(double position) {
        this.boxPositions[BoxPosition.SILVER_COLLECTION.ordinal()] = position;
        if (adjustmentMode) this.boxServo.setPosition(position);
    }

    @Adjustable(min = 0, max = 250, step = 1)
    public double getBoxDump() {
        return this.boxPositions[BoxPosition.DUMP.ordinal()];
    }
    public void setBoxDump(double position) {
        this.boxPositions[BoxPosition.DUMP.ordinal()] = position;
        if (adjustmentMode) this.boxServo.setPosition(position);
    }

    @Adjustable(min = 0.0, max = 1.0, step = 0.01)
    public double getSweeperInPower() {
        return sweeperInPower;
    }
    public void setSweeperInPower(double power) {
        this.sweeperInPower = power;
        if (adjustmentMode) {
            this.sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.sweeperMotor.setPower(power);
        }
    }

    @Adjustable(min = 0.0, max = 1.0, step = 0.01)
    public double getSweeperOutPower() {
        return sweeperOutPower;
    }
    public void setSweeperOutPower(double power) {
        this.sweeperOutPower = power;
        if (adjustmentMode) {
            this.sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.sweeperMotor.setPower(power);
        }
    }

    // getter and setter use double instead of int due to @Adjustable only supporting doubles
    @Adjustable(min = 1, max = 1000, step = 1)
    public double getSweeperHalfRotation() {
        return sweeperHalfRotation;
    }
    public void setSweeperHalfRotation(double position) {
        this.sweeperHalfRotation = (int) Math.floor(position);
        if (adjustmentMode) {
            this.sweeperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.sweeperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.sweeperMotor.setTargetPosition(sweeperHalfRotation);
            this.sweeperMotor.setPower(sweeperInPower);
        }
    }

    @Adjustable(min = 0.0, max = 8000.0, step = 1.0)
    public double getSliderExtended() {
        return sliderExtended;
    }
    public void setSliderExtended(double sliderExtended) {
        this.sliderExtended = sliderExtended;
        if (adjustmentMode) {
            this.sliderMotor.setTargetPosition((int) Math.floor(sliderExtended));
            this.sliderMotor.setPower(this.sliderPower);
        }
    }

    @Adjustable(min = 0.0, max = 8000.0, step = 1.0)
    public double getSliderSafe() {
        return sliderSafe;
    }
    public void setSliderSafe(double sliderSafe) {
        this.sliderSafe = sliderSafe;
        if (adjustmentMode) {
            this.sliderMotor.setTargetPosition((int) Math.floor(sliderSafe));
            this.sliderMotor.setPower(this.sliderPower);
        }
    }

    @Adjustable(min = 0.0, max = 8000.0, step = 1.0)
    public double getSliderDump() {
        return sliderDump;
    }
    public void setSliderDump(double sliderDump) {
        this.sliderDump = sliderDump;
        if (adjustmentMode) {
            this.sliderMotor.setTargetPosition((int) Math.floor(sliderDump));
            this.sliderMotor.setPower(this.sliderPower);
        }
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
        boxServo.setPosition(this.boxPositions[BoxPosition.INITIAL.ordinal()]);
        resetMotor(sweeperMotor);
        resetMotor(sliderMotor);
    }

    /**
     * Rotates the collection box to the specified position
     * @param position {@link BoxPosition}
     * @return operation showing whether rotation is complete
     */
    public Operation rotateBox(BoxPosition position) {
        double targetPosition = this.boxPositions[position.ordinal()];
        double adjustment = Math.abs(boxServo.getPosition() - targetPosition);
        if (arePositionsCompatible(targetPosition, 1.0 * this.sliderMotor.getCurrentPosition())) {
            boxServo.setPosition(targetPosition);
        } else {
            adjustment = 0;
        }
        final long doneBy = System.currentTimeMillis() + Math.round(2 * adjustment);
        return new Operation() {
            @Override
            public boolean isFinished() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    /**
     * Rotates the sweeper half a circle for either intake or pushing out
     * @param intake <code>true</code> for intake, <code>false</code> for pushing out
     * @return operation showing whether rotation is complete
     */
    public Operation rotateSweeper(boolean intake) {
        this.sweeperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.sweeperMotor.setTargetPosition(this.sweeperMotor.getTargetPosition()
                + (intake ? 1 : -1) * sweeperHalfRotation);
        this.sweeperMotor.setPower(intake ? sweeperInPower : (-1 * sweeperOutPower));
        return new Operation() {
            @Override
            public boolean isFinished() {
                return sweeperMotor.isBusy();
            }
        };
    }

    /**
     * Moves the slider to given position
     * @param position to move slider to
     * @return operation showing whether movement is complete
     */
    public Operation moveSlider(double position) {
        if (position < this.sliderContracted) {
            throw new IllegalArgumentException("Slider position cannot be less than [sliderContracted]");
        }
        if (position > this.sliderExtended) {
            throw new IllegalArgumentException("Slider position cannot be greater than [sliderExtended]");
        }

        if (arePositionsCompatible(this.boxServo.getPosition(), position)) {
            this.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.sliderMotor.setTargetPosition((int) Math.round(position));
            this.sliderMotor.setPower(this.sliderPower);
        }
        return new Operation() {
            @Override
            public boolean isFinished() {
                return sliderMotor.isBusy();
            }
        };
    }

    private boolean arePositionsCompatible(double boxPosition, double sliderPosition) {
        boolean collectionMode = boxPosition==getBoxGoldCollection()
                || boxPosition==getBoxSilverCollection();
        if (collectionMode && sliderPosition < getSliderSafe()) return false;

        if (boxPosition == getBoxDump() && sliderPosition!= getSliderDump()) return false;

        return true;
    }

    private void resetMotor(DcMotor motor) {
        motor.setPower(0.0d);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


}
