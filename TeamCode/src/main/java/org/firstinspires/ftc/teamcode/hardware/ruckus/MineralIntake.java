package org.firstinspires.ftc.teamcode.hardware.ruckus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.components.Operation;
import org.firstinspires.ftc.teamcode.components.SwerveChassis;
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

    public enum BoxPosition {
        INITIAL(0),
        GOLD_COLLECTION(100),
        SILVER_COLLECTION(120),
        DUMP(-100);

        double value;
        BoxPosition(double value) {
            this.value = value;
        }
    };

    private DcMotor sweeperMotor;
    private DcMotor sliderMotor;
    private AdjustableServo boxServo;
    private boolean adjustmentMode = false;

    // sweeper intake / push out power values
    private double sweeperInPower = 0.2;
    private double sweeperOutPower = 0.3;
    // encoder value for sweeper rotating half circle
    private int sweeperHalfRotation = 240;
    // maximum encoder value for sweeper motor
    private int sweeperMaxPosition = 10000;

    private int sweeperLastPosition = 0;
    private int sweeperOverflowCount = 0;

    // slider encoder positions
    private int sliderContracted = 0; // contracted
    private int sliderExtended = 1400; // fully extended
    // minimally extended position that allows for box to be rotated to one of the collection positions
    private int sliderSafe = 700;
    private int sliderDump = 600; // allows box to rest on back bracket and dump
    private double sliderPower = 0.2; // TBD

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
            this.boxServo.setPosition(BoxPosition.INITIAL.value);
            debug("Adjustment: OFF, box: %.1f, sweeper: %s / %d, slider: %s", boxServo.getPosition(),
                    sweeperMotor.getMode(), sweeperMotor.getCurrentPosition(),
                    sliderMotor.getMode(), sliderMotor.getCurrentPosition()
            );
        }
    }

    @Adjustable(min = 90, max = 125, step = 1)
    public double getBoxGoldCollection() {
        return BoxPosition.GOLD_COLLECTION.value;
    }
    public void setBoxGoldCollection(double position) {
        BoxPosition.GOLD_COLLECTION.value = position;
        if (adjustmentMode) this.boxServo.setPosition(position);
    }

    @Adjustable(min = 90, max = 125, step = 1)
    public double getBoxSilverCollection() {
        return BoxPosition.SILVER_COLLECTION.value;
    }
    public void setBoxSilverCollection(double position) {
        BoxPosition.SILVER_COLLECTION.value = position;
        if (adjustmentMode) this.boxServo.setPosition(position);
    }

    @Adjustable(min = -125, max = -75, step = 1)
    public double getBoxDump() {
        return BoxPosition.DUMP.value;
    }
    public void setBoxDump(double position) {
        BoxPosition.DUMP.value = position;
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

    @Adjustable(min = 0, max = 1000, step = 5)
    public int getSweeperHalfRotation() {
        return sweeperHalfRotation;
    }
    public void setSweeperHalfRotation(int position) {
        this.sweeperHalfRotation = position;
        if (adjustmentMode) {
            debug("Sweeper Adjustment: ON, position: %d / %d, power: %.3f / %.3f", sweeperMotor.getCurrentPosition(), position, sweeperMotor.getPower(), sweeperInPower);
            this.sweeperMotor.setTargetPosition(sweeperHalfRotation);
            this.sweeperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.sweeperMotor.setPower(sweeperInPower);
            debug("Sweeper Adjustment Running: ON, position: %d / %d, power: %.3f, mode: %s", sweeperMotor.getCurrentPosition(), sweeperMotor.getTargetPosition(), sweeperMotor.getPower(), sweeperMotor.getMode());
        }
    }

    @Adjustable(min = 0, max = 20000, step = 5)
    public int getSweeperMaxPosition() {
        return sweeperMaxPosition;
    }
    public void setSweeperMaxPosition(int position) {
        this.sweeperMaxPosition = position;
    }

    public int getSliderContracted() {
        return sliderContracted;
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
            this.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        sliderMotor.setDirection(DcMotor.Direction.FORWARD);

        configuration.register(this);
    }

    public void reset() {
        boxServo.setPosition(BoxPosition.INITIAL.value);
        resetMotor(sweeperMotor);
        resetMotor(sliderMotor);
        this.sweeperOverflowCount = 0;
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
        double adjustment = Math.abs(boxServo.getPosition() - position.value);
        if (arePositionsCompatible(position.value, this.sliderMotor.getCurrentPosition())) {
            boxServo.setPosition(position.value);
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
            if (Math.abs(boxServo.getPosition() - position.value) < 0.1) {
                return position;
            }
        }
        return BoxPosition.INITIAL;
    }

    public enum SweeperMode { INTAKE, PUSH_OUT, VERTICAL_STOP, HORIZONTAL_STOP };
    /**
     * Rotates or parks the sweeper according to the mode specified
     * @param mode direction to rotate or position to park the sweeper in
     */
    public void rotateSweeper(SweeperMode mode) {
        double power = mode==SweeperMode.INTAKE ? sweeperInPower : (-1 * sweeperOutPower);
        if ((mode==SweeperMode.INTAKE) || (mode==SweeperMode.PUSH_OUT)) {
            int currentPosition = this.sweeperMotor.getCurrentPosition();
            // determine if sweeper position has just crossed the maximum value
            int difference = currentPosition - this.sweeperLastPosition;
            if (difference * power < 0) {
                this.sweeperOverflowCount += difference < 0 ? 1 : -1;
                debug("rotateSweeper() overflow count: %d, last: %d, current: %d, power: %.2f",
                        this.sweeperOverflowCount, this.sweeperLastPosition,
                        currentPosition, power
                );
            }
            this.sweeperLastPosition = currentPosition;
            this.sweeperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.sweeperMotor.setPower(power);
        } else {
            this.sweeperMotor.setPower(0);
            // find closest position evenly divisible by half rotation
            int offset = (this.sweeperOverflowCount * this.sweeperMaxPosition) % this.sweeperHalfRotation;
            int targetPosition = Math.round(1.0f * (this.sweeperMotor.getCurrentPosition() + offset) / this.sweeperHalfRotation) * this.sweeperHalfRotation;
            if (mode==SweeperMode.HORIZONTAL_STOP) {
                // adjust target position by quarter rotation
                if (targetPosition > 0) {
                    targetPosition += this.sweeperHalfRotation / 2;
                } else {
                    targetPosition -= this.sweeperHalfRotation / 2;
                }
            }
            // make sure target position does not exceed overflow point
            if (targetPosition > this.sweeperMaxPosition) {
                targetPosition -= this.sweeperHalfRotation;
            } else if (targetPosition < -1 * this.sweeperMaxPosition) {
                targetPosition += this.sweeperHalfRotation;
            }
            debug("rotateSweeper() park: %d, overflow: %d, offset: %d, max: %d",
                    targetPosition, this.sweeperOverflowCount,
                    offset, this.sweeperMaxPosition
            );
            this.sweeperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.sweeperMotor.setTargetPosition(targetPosition);
            this.sweeperLastPosition = targetPosition;
            this.sweeperMotor.setPower(power);
        }
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

    public void stopSlider() {
        this.sliderMotor.setPower(0);
    }

    public int getSliderCurrent() {
        return this.sliderMotor.getCurrentPosition();
    }

    public int getSliderTarget() {
        return this.sliderMotor.getTargetPosition();
    }

    /**
     * Set up telemetry lines for intake metrics
     * Shows encoder values for slider / sweeper and box position
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        line.addData("Slide", "%d", new Func<Integer>() {
            @Override
            public Integer value() {
                return sliderMotor.getCurrentPosition();
            }
        });
        line.addData("Sweep", "%d", new Func<Integer>() {
            @Override
            public Integer value() {
                return sweeperMotor.getCurrentPosition();
            }
        });
        line.addData("Box", "%.1f", new Func<Double>() {
            @Override
            public Double value() {
                return boxServo.getPosition();
            }
        });
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
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
