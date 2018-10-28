package org.firstinspires.ftc.teamcode.hardware.ruckus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
 * sweeper (motor), sw_slider (motor), sv_sw_box (servo)
 */
public class MineralIntake extends Logger<MineralIntake> implements Configurable {

    public enum BoxPosition { INITIAL, GOLD_COLLECTION, SILVER_COLLECTION, DUMP };

    private DcMotor sweeperMotor;
    private DcMotor sliderMotor;
    private AdjustableServo boxServo;
    private boolean adjustmentMode = false;

    // adjustable servo positions in order of BoxPosition constants
    private double[] boxPositions = { 0, 100, 120, -100 };

    // sweeper intake / push out power values
    private double sweeperInPower = 0.2;
    private double sweeperOutPower = 0.3;
    private int sweeperHalfRotation = 250; // TBD

    // slider encoder positions
    private int sliderContracted = 0; // contracted
    private int sliderExtended = 70; // fully extended
    // minimally extended position that allows for box to be rotated to one of the collection positions
    private int sliderSafe = 35;
    private int sliderDump = 30; // allows box to rest on back bracket and dump
    private double sliderPower = 0.1; // TBD

    @Override
    public String getUniqueName() {
        return "intake";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        if (this.adjustmentMode == on) return;
        this.adjustmentMode = on;
        if (on) {
            debug("Adjustment: ON, box: %.1f, sweeper: %s / %d, slider: %s", boxServo.getPosition(),
                    sweeperMotor.getMode(), sweeperMotor.getCurrentPosition(),
                    sliderMotor.getMode(), sliderMotor.getCurrentPosition()
            );
        } else {
            resetMotor(this.sliderMotor);
            resetMotor(this.sweeperMotor);
            this.boxServo.setPosition(boxPositions[BoxPosition.INITIAL.ordinal()]);
            debug("Adjustment: OFF, box: %.1f, sweeper: %s / %d, slider: %s", boxServo.getPosition(),
                    sweeperMotor.getMode(), sweeperMotor.getCurrentPosition(),
                    sliderMotor.getMode(), sliderMotor.getCurrentPosition()
            );
        }
    }

    @Adjustable(min = 90, max = 125, step = 1)
    public double getBoxGoldCollection() {
        return this.boxPositions[BoxPosition.GOLD_COLLECTION.ordinal()];
    }
    public void setBoxGoldCollection(double position) {
        this.boxPositions[BoxPosition.GOLD_COLLECTION.ordinal()] = position;
        if (adjustmentMode) this.boxServo.setPosition(position);
    }

    @Adjustable(min = 90, max = 125, step = 1)
    public double getBoxSilverCollection() {
        return this.boxPositions[BoxPosition.SILVER_COLLECTION.ordinal()];
    }
    public void setBoxSilverCollection(double position) {
        this.boxPositions[BoxPosition.SILVER_COLLECTION.ordinal()] = position;
        if (adjustmentMode) this.boxServo.setPosition(position);
    }

    @Adjustable(min = -125, max = -75, step = 1)
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
            this.sweeperMotor.setTargetPosition(0);
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
            this.sweeperMotor.setTargetPosition(0);
            this.sweeperMotor.setPower(power);
        }
    }

    @Adjustable(min = 1, max = 1000, step = 5)
    public int getSweeperHalfRotation() {
        return sweeperHalfRotation;
    }
    public void setSweeperHalfRotation(int position) {
        this.sweeperHalfRotation = position;
        if (adjustmentMode) {
            debug("Sweeper Adjustment: ON, position: %d / %d, power: %.3f / %.3f", sweeperMotor.getCurrentPosition(), position, sweeperMotor.getPower(), sweeperInPower);
            this.sweeperMotor.setTargetPosition(sweeperHalfRotation);
            this.sweeperMotor.setPower(sweeperInPower);
            debug("Sweeper Adjustment Running: ON, position: %d / %d, power: %.3f, mode: %s", sweeperMotor.getCurrentPosition(), sweeperMotor.getTargetPosition(), sweeperMotor.getPower(), sweeperMotor.getMode());
        }
    }

    @Adjustable(min = 0.0, max = 8000.0, step = 1.0)
    public int getSliderExtended() {
        return sliderExtended;
    }
    public void setSliderExtended(int sliderExtended) {
        this.sliderExtended = sliderExtended;
        if (adjustmentMode) {
            debug("Slider Adjustment: ON, position: %d / %d, power: %.3f / %.3f", sliderMotor.getCurrentPosition(), sliderExtended, sliderMotor.getPower(), sliderPower);
            this.sliderMotor.setTargetPosition(this.sliderExtended);
            this.sliderMotor.setPower(this.sliderPower);
        }
    }

    @Adjustable(min = 0.0, max = 8000.0, step = 1.0)
    public int getSliderSafe() {
        return sliderSafe;
    }
    public void setSliderSafe(int sliderSafe) {
        this.sliderSafe = sliderSafe;
        if (adjustmentMode) {
            debug("Slider Adjustment: ON, position: %d / %d, power: %.3f / %.3f", sliderMotor.getCurrentPosition(), sliderSafe, sliderMotor.getPower(), sliderPower);
            this.sliderMotor.setTargetPosition(sliderSafe);
            this.sliderMotor.setPower(this.sliderPower);
            debug("Slider Adjustment Running: ON, position: %d / %d, power: %.3f, mode: %s", sliderMotor.getCurrentPosition(), sliderMotor.getTargetPosition(), sliderMotor.getPower(), sliderMotor.getMode());
        }
    }

    @Adjustable(min = 0.0, max = 8000.0, step = 1.0)
    public int getSliderDump() {
        return sliderDump;
    }
    public void setSliderDump(int sliderDump) {
        this.sliderDump = sliderDump;
        if (adjustmentMode) {
            debug("Slider Adjustment: ON, position: %d / %d, power: %.3f / %.3f", sliderMotor.getCurrentPosition(), sliderDump, sliderMotor.getPower(), sliderPower);
            this.sliderMotor.setTargetPosition(sliderDump);
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
        boxServo = new AdjustableServo(-125, 125).configureLogging(
                logTag + ":servo" , logLevel
        );
        boxServo.configure(configuration.getHardwareMap(), "sv_sw_box");
        configuration.register(boxServo);

        sweeperMotor = configuration.getHardwareMap().tryGet(DcMotor.class, "sweeper");
        sweeperMotor.setDirection(DcMotor.Direction.FORWARD);

        sliderMotor = configuration.getHardwareMap().tryGet(DcMotor.class, "sw_slider");
        sliderMotor.setDirection(DcMotor.Direction.REVERSE);

        configuration.register(this);
    }

    public void reset() {
        boxServo.setPosition(this.boxPositions[BoxPosition.INITIAL.ordinal()]);
        resetMotor(sweeperMotor);
        resetMotor(sliderMotor);
        debug("Reset mineral intake, box: %.1f, sweeper: %s / %d, slider: %s / %d", boxServo.getPosition(),
                sweeperMotor.getMode(), sweeperMotor.getCurrentPosition(),
                sliderMotor.getMode(), sliderMotor.getCurrentPosition()
        );
    }

    /**
     * Rotates the collection box to the specified position
     * @param position {@link BoxPosition}
     * @return operation showing whether rotation is complete
     */
    public Operation rotateBox(BoxPosition position) {
        double targetPosition = this.boxPositions[position.ordinal()];
        double adjustment = Math.abs(boxServo.getPosition() - targetPosition);
        if (arePositionsCompatible(targetPosition, this.sliderMotor.getCurrentPosition())) {
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

    public BoxPosition getBoxPosition() {
        for (BoxPosition position : BoxPosition.values()) {
            if (boxServo.getPosition()==boxPositions[position.ordinal()]) {
                return position;
            }
        }
        return BoxPosition.INITIAL;
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
    public Operation moveSlider(int position) {
        if (position < this.sliderContracted) {
            throw new IllegalArgumentException("Slider position cannot be less than [sliderContracted]");
        }
        if (position > this.sliderExtended) {
            throw new IllegalArgumentException("Slider position cannot be greater than [sliderExtended]");
        }

        if (arePositionsCompatible(this.boxServo.getPosition(), position)) {
            this.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.sliderMotor.setTargetPosition(position);
            this.sliderMotor.setPower(this.sliderPower);
        }
        return new Operation() {
            @Override
            public boolean isFinished() {
                return sliderMotor.isBusy();
            }
        };
    }

    public int getSliderCurrent() {
        return this.sliderMotor.getCurrentPosition();
    }

    public int getSliderTarget() {
        return this.sliderMotor.getTargetPosition();
    }

    private boolean arePositionsCompatible(double boxPosition, int sliderPosition) {
        boolean collectionMode = boxPosition==getBoxGoldCollection()
                || boxPosition==getBoxSilverCollection();
        if (collectionMode && sliderPosition < getSliderSafe()) return false;

        if (boxPosition == getBoxDump() && sliderPosition!= getSliderDump()) return false;

        return true;
    }

    private void resetMotor(DcMotor motor) {
        motor.setPower(0d);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
