package org.firstinspires.ftc.teamcode.hardware.ruckus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

/**
 * MineralIntake consists of 2 motors (1 controlling the sweeper, and 1 controlling the slider)
 * and 1 servo controlling the angle of the collection box.<br />
 * Expected hardware configuration is:<br />
 * Servos: sv_sw_box (intake box gate) and sv_box_lift (intake box lift, multiple rotations)
 * Motors: sweeper and sw_slider<br />
 */
public class MineralIntake extends Logger<MineralIntake> implements Configurable {

    public enum SweeperMode {INTAKE, PUSH_OUT, VERTICAL_STOP, HORIZONTAL_STOP}

    // down and up positions for the box lift
    // actual servo positions are configured via <code>AdjustableServo</code>
    public static final double LIFT_DOWN = 0.0;
    public static final double LIFT_CENTER = 0.5;
    public static final double LIFT_UP = 1.0;

    // open and closed positions for the box gate
    // actual servo positions are configured via <code>AdjustableServo</code>
    public static final double GATE_OPEN = 1.0;
    public static final double GATE_CLOSED = 0.0;

    private DcMotor sweeperMotor;
    private DcMotor sliderMotor;
    private AdjustableServo boxLiftServo;
    private AdjustableServo boxGateServo;
    private boolean adjustmentMode = false;

    private double sweeperInPower = 0.99;
    private double sweeperOutPower = 0.7;
    // encoder value for sweeper rotating half circle
    private int sweeperHalfRotation = 180;


    // slider encoder positions
    private int sliderOffset = 0; // offset will be set to sliderInitOut when manual reset
    private int iSliderContracted = 0; // contracted
    private int iSliderExtended = 2100; // fully extended
    private int iSliderDump = 400; // position to dump minerals into delivery box
    private int iSliderInitOut = 370; // position for initial TeleOp out, lifter just out
    private int iSliderSafeLiftPos = 967;
    private int iSliderMinSweep = 1000; // pos for min sweeping
    private int iSliderAutoPark = 1000;

    private int sliderContracted = iSliderContracted; // contracted
    private int sliderExtended = iSliderExtended; // fully extended
    private int sliderDump = iSliderDump; // position to dump minerals into delivery box
    private int sliderInitOut = iSliderInitOut; // position for initial TeleOp out, lifter just out
    private int sliderSafeLiftPos = iSliderSafeLiftPos;
    private int sliderMinSweep = iSliderMinSweep; // pos for min sweeping
    private int sliderAutoPark = iSliderAutoPark; // position for Auto Out parking;
    private double sliderPower = 0.6; // TBD

    public void syncSliderEncoder() {
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderOffset = -400; // offset will be set to -sliderInitOut when manual reset
        sliderContracted = sliderOffset; // contracted
        sliderExtended = sliderOffset + iSliderExtended; // fully extended
        sliderDump = sliderOffset + iSliderDump; // position to dump minerals into delivery box
        sliderInitOut = sliderOffset + iSliderInitOut; // position for initial TeleOp out, lifter just out
        sliderSafeLiftPos = sliderOffset + iSliderSafeLiftPos;
        sliderMinSweep = sliderOffset + iSliderMinSweep; // pos for min sweeping
        sliderAutoPark = sliderOffset + iSliderAutoPark; // position for Auto Out parking;
    }

    @Override
    public String getUniqueName() {
        return "intake";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        if (this.adjustmentMode == on) return;
        this.adjustmentMode = on;
        if (on) {
            debug("Adjustment: ON, lift: %.2f, gate: %.1f, sweeper: %s / %d, slider: %s",
                    boxLiftServo.getPosition(), boxGateServo.getPosition(),
                    sweeperMotor.getMode(), sweeperMotor.getCurrentPosition(),
                    sliderMotor.getMode(), sliderMotor.getCurrentPosition()
            );
        } else {
            this.sliderMotor.setPower(0);
            this.sweeperMotor.setPower(0);
            this.boxLiftServo.setPosition(LIFT_CENTER);
            this.boxGateServo.setPosition(GATE_CLOSED);
            debug("Adjustment: OFF, lift: %.2f, gate: %.1f, sweeper: %s / %d, slider: %s",
                    boxLiftServo.getPosition(), boxGateServo.getPosition(),
                    sweeperMotor.getMode(), sweeperMotor.getCurrentPosition(),
                    sliderMotor.getMode(), sliderMotor.getCurrentPosition()
            );
        }
    }

    public void boxLiftUp() {
        this.boxLiftServo.setPosition(LIFT_UP);
    }

    public void boxLiftCenter() {
        this.boxLiftServo.setPosition(LIFT_CENTER);
    }

    public void boxLiftDown() {
        this.boxLiftServo.setPosition(LIFT_DOWN);
    }

    public void boxLiftDownCombo() {
        final String taskName = "boxLiftDown";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                if (sliderMotor.getCurrentPosition() < sliderMinSweep) {
                    moveSliderFast(sliderMinSweep);
                }
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (getSliderCurrent() > (sliderMinSweep - 10));
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                final Progress boxProgress = moveBox(false, false);
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return boxProgress.isDone();
                    }
                };
            }
        }, taskName);
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
            this.sweeperMotor.setPower(-1 * power);
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

    public int getSliderContracted() {
        return sliderContracted;
    }

    public int getSliderSafeLiftPos() {
        return sliderSafeLiftPos;
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
            this.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.sliderMotor.setPower(this.sliderPower);
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
            this.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.sliderMotor.setPower(this.sliderPower);
        }
    }

    public int getSliderMinSweep() {
        return sliderMinSweep;
    }

    public int getSliderInitOut() {
        return sliderInitOut;
    }

    public int getSliderAutoPark() {
        return sliderAutoPark;
    }

    public void setSliderAutoPark() {
        moveSlider(sliderAutoPark);
    }

    @Adjustable(min = 0.0, max = 1.0, step = 0.01)
    public double getSliderPower() {
        return sliderPower;
    }

    public void setSliderPower(double sliderPower) {
        this.sliderPower = sliderPower;
    }

    public void configure(Configuration configuration) {
        boxLiftServo = new AdjustableServo(LIFT_DOWN, LIFT_UP).configureLogging(
                logTag + ":boxLift", logLevel
        );
        boxLiftServo.configure(configuration.getHardwareMap(), "sv_box_lift");
        configuration.register(boxLiftServo);

        boxGateServo = new AdjustableServo(GATE_CLOSED, GATE_OPEN).configureLogging(
                logTag + ":boxGate", logLevel
        );
        boxGateServo.configure(configuration.getHardwareMap(), "sv_sw_box");
        configuration.register(boxGateServo);

        sweeperMotor = configuration.getHardwareMap().tryGet(DcMotor.class, "sweeper");
        sweeperMotor.setDirection(DcMotor.Direction.REVERSE);

        sliderMotor = configuration.getHardwareMap().tryGet(DcMotor.class, "sw_slider");
        sliderMotor.setDirection(DcMotor.Direction.REVERSE);

        configuration.register(this);
    }

    public void reset(boolean auto) {
        boxLiftServo.setPosition(LIFT_CENTER);
        boxGateServo.setPosition(GATE_CLOSED);
        resetMotor(sweeperMotor);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (auto) {
            resetMotor(sliderMotor);
        }
        debug("Reset mineral intake, lift: %.2f, gate: %.1f, sweeper: %s / %d, slider: %s / %d",
                boxLiftServo.getPosition(), boxGateServo.getPosition(),
                sweeperMotor.getMode(), sweeperMotor.getCurrentPosition(),
                sliderMotor.getMode(), sliderMotor.getCurrentPosition()
        );
    }

    /**
     * Moves collection box to specified position (up or down)
     *
     * @param up <code>true</code> to move box up, <code>false</code> to move box down
     * @return estimated progress
     */
    public Progress moveBox(boolean up, boolean center) {
        // !up always go LIFT_DOWN
        // up go LIFT_CENTER , then LIFT_UP
        double target = up ? LIFT_UP : LIFT_DOWN;
        if ((up && isBoxDown()) || center) {
            target = LIFT_CENTER;
        }
        double adjustment = Math.abs(boxLiftServo.getPosition() - target);
        debug("moveBox(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from up to down takes 2 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(600 * adjustment);
        boxLiftServo.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void boxUp() {
        boxLiftServo.setPosition(LIFT_UP);
    }

    public void boxDown() {
        boxLiftServo.setPosition(LIFT_DOWN);
    }

    public void boxCenter() {
        boxLiftServo.setPosition(LIFT_CENTER);
    }

    public boolean isBoxUp() {
        return Math.abs(boxLiftServo.getPosition() - LIFT_UP) < 0.01;
    }

    public boolean isBoxDown() {
        return Math.abs(boxLiftServo.getPosition() - LIFT_DOWN) < 0.01;
    }

    public boolean isBoxCenter() {
        return Math.abs(boxLiftServo.getPosition() - LIFT_CENTER) < 0.01;
    }

    /**
     * Moves collection box gate to specified position (open or closed)
     *
     * @param open <code>true</code> to open the gate, <code>false</code> to close it
     * @return estimated progress
     */
    public Progress moveGate(boolean open) {
        double target = open ? GATE_OPEN : GATE_CLOSED;
        double adjustment = Math.abs(boxGateServo.getPosition() - target);
        debug("moveGate(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from open to closed takes 0.25 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(250 * adjustment);
        boxGateServo.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public boolean isGateOpen() {
        return Math.abs(boxGateServo.getPosition() - GATE_OPEN) < 0.01;
    }

    public void mineralDumpCombo() {
        final String taskName = "mineralDump";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                final Progress boxProgress = moveBox(true, true);
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return boxProgress.isDone();
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                moveSliderFast(getSliderDump());
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return Math.abs(getSliderCurrent() - getSliderDump()) < 30;
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGate(true);
            }
        }, taskName);
    }

    /**
     * Rotates or parks the sweeper according to the mode specified
     *
     * @param mode direction to rotate or position to park the sweeper in
     */
    public void rotateSweeper(SweeperMode mode) throws InterruptedException {
        if ((mode == SweeperMode.INTAKE) || (mode == SweeperMode.PUSH_OUT)) {
            this.sweeperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            this.sweeperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double power = mode == SweeperMode.INTAKE ? sweeperInPower : (-1 * sweeperOutPower);
            this.sweeperMotor.setPower(power);
            return;
        }

        this.sweeperMotor.setPower(0);
        // wait for the motor to come to a stop for up to a second
        int position = this.sweeperMotor.getCurrentPosition();
        int count = 20;
        while (count-- > 0) {
            Thread.sleep(50);
            int newPosition = this.sweeperMotor.getCurrentPosition();
            if (Math.abs(newPosition - position) < 2) break;
            debug("rotateSweeper(): wait pos=%d, new=%d", position, newPosition);
            position = newPosition;
        }
        int targetPosition = 0;
        if (mode == SweeperMode.HORIZONTAL_STOP) {
            targetPosition = (int) Math.round(Math.floor(1.0d * position / this.sweeperHalfRotation)) - this.sweeperHalfRotation / 2;
        } else {
            targetPosition = (int) Math.round(Math.ceil(1.0d * position / this.sweeperHalfRotation)) * this.sweeperHalfRotation;
        }
        debug("rotateSweeper(): pos=%d, target=%d", position, targetPosition);

        // position is within 6 degrees of target; do not adjust it any further
        if (Math.abs(targetPosition - position) < this.sweeperHalfRotation / 30) return;

        this.sweeperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.sweeperMotor.setTargetPosition(targetPosition);
        this.sweeperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double power = mode == SweeperMode.VERTICAL_STOP ? sweeperInPower : (-1 * sweeperOutPower);
        this.sweeperMotor.setPower(power / 3);

        // wait for the motor to come to a stop for up to a second
        count = 20;
        while (this.sweeperMotor.isBusy() && count-- > 0) {
            Thread.sleep(50);
            debug("rotateSweeper(): wait2 pos=%d", this.sweeperMotor.getCurrentPosition());
        }
        this.sweeperMotor.setPower(0);
    }

    public void sweeperIn() {
        this.sweeperMotor.setPower(sweeperInPower);
    }

    public void sweeperOut() {
        this.sweeperMotor.setPower(-1 * sweeperOutPower);
    }

    public void sweeperIn(double power) {
        this.sweeperMotor.setPower(Math.min(1,Math.abs(power)));
    }

    public void sweeperOut(double power) {
        this.sweeperMotor.setPower(-Math.min(1,Math.abs(power)));
    }

    /**
     * Moves the slider to given position
     *
     * @param position to move slider to
     * @return operation showing whether movement is complete
     */
    public Progress moveSlider(int position) {
//        if (position < this.sliderContracted) {
//            throw new IllegalArgumentException("Slider position cannot be less than [sliderContracted]");
//        }
//        if (position > this.sliderExtended) {
//            throw new IllegalArgumentException("Slider position cannot be greater than [sliderExtended]");
//        }

        this.sliderMotor.setTargetPosition(position);
        this.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.sliderMotor.setPower(this.sliderPower);
        return new Progress() {
            @Override
            public boolean isDone() {
                return sliderMotor.isBusy();
            }
        };
    }

    public Progress moveSliderFast(int position) {
//        if (position < this.sliderContracted) {
//            throw new IllegalArgumentException("Slider position cannot be less than [sliderContracted]");
//        }
//        if (position > this.sliderExtended) {
//            throw new IllegalArgumentException("Slider position cannot be greater than [sliderExtended]");
//        }

        this.sliderMotor.setTargetPosition(position);
        this.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // this.sliderMotor.setPower(0.9);
        this.sliderMotor.setPower(this.sliderPower);
        return new Progress() {
            @Override
            public boolean isDone() {
                return sliderMotor.isBusy();
            }
        };
    }

    public void adjustSlider(boolean extend) {
        this.sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.sliderMotor.setPower(this.sliderPower * (extend ? 1 : -1));
    }

    public void stopSlider() {
        this.sliderMotor.setPower(0);
    }

    public void stopSweeper() {
        this.sweeperMotor.setPower(0);
    }

    public int getSliderCurrent() {
        return this.sliderMotor.isBusy() ? this.sliderMotor.getCurrentPosition() : this.sliderMotor.getTargetPosition();
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
        line.addData("Box", new Func<String>() {
            @Override
            public String value() {
                return (isBoxUp() ? "Up" : "Down") + "/"
                        + (isGateOpen() ? "Open" : "Closed");
            }
        });
        line.addData("Slide", new Func<String>() {
            @Override
            public String value() {
                int position = sliderMotor.getTargetPosition();
                String name = "0";
                if (position == sliderDump) name = "Dump";
                if (position == sliderExtended) name = "Ext";
                return String.format("%s:%d", name, sliderMotor.getCurrentPosition());
            }
        });
        line.addData("Sweep", "%d", new Func<Integer>() {
            @Override
            public Integer value() {
                return sweeperMotor.getCurrentPosition();
            }
        });
    }

    private void resetMotor(DcMotor motor) {
        motor.setPower(0d);
        motor.setTargetPosition(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
