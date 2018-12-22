package org.firstinspires.ftc.teamcode.hardware.titan;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

/**
 * MineralArm consists of 2 motors (1 arm shoulder, and 1 controlling the slider)
 * and 1 servo controlling the angle of the collection box.<br />
 * Expected hardware configuration is:<br />
 *   Servos:  sweeper (continuous servo), gate
 *   Motors: shoulder and arm_slider <br />
 */
public class MineralArm extends Logger<MineralArm> implements Configurable {

    public enum SweeperMode { INTAKE, PUSH_OUT, VERTICAL_STOP, HORIZONTAL_STOP }

    // open and closed positions for the box gate
    // actual servo positions are configured via <code>AdjustableServo</code>
    public static final double GATE_OPEN = 0.0;
    public static final double GATE_CLOSED = 1.0;

    public static final double SWEEPER_OFF = 0.5;
    public static final double SWEEPER_IN = 1.0;
    public static final double SWEEPER_OUT = 0.0;

    private DcMotor shoulder;
    private DcMotor armSlider;
    private Servo sweeperServo;
    private AdjustableServo gate;
    private boolean adjustmentMode = false;

    // slider encoder positions
    private int sliderContracted = 0; // contracted
    private int sliderExtended = 1520; // fully extended
    private int sliderDump = 300; // position to dump minerals into delivery box
    private int sliderInitOut = 450; // position for initial TeleOp out
    private int sliderAutoPark = 650; // position for Auto Out parking;
    private double sliderPower = 0.6;
    private double shoulderPower = 0.8;

    @Override
    public String getUniqueName() {
        return "intake";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        if (this.adjustmentMode == on) return;
        this.adjustmentMode = on;
        if (on) {
            debug("Adjustment: ON, sweeper: %.2f, gate: %.1f, shoulder: %s / %d, slider: %s / %d",
                    sweeperServo.getPosition(), gate.getPosition(),
                    shoulder.getMode(), shoulder.getCurrentPosition(),
                    armSlider.getMode(), armSlider.getCurrentPosition()
            );
        } else {
            this.armSlider.setPower(0);
            this.shoulder.setPower(0);
            this.sweeperServo.setPosition(SWEEPER_OFF);
            this.gate.setPosition(GATE_CLOSED);
            debug("Adjustment: OFF, sweeper: %.2f, gate: %.1f, shoulder: %s / %d, slider: %s / %d",
                    sweeperServo.getPosition(), gate.getPosition(),
                    shoulder.getMode(), shoulder.getCurrentPosition(),
                    armSlider.getMode(), armSlider.getCurrentPosition()
            );
        }
    }
    public void sweeperIn() { this.sweeperServo.setPosition(SWEEPER_IN); }
    public void sweeperOut() {
        this.sweeperServo.setPosition(SWEEPER_OUT);
    }

    public void stopSweeper() {
        this.sweeperServo.setPosition(SWEEPER_OFF);
    }

    @Adjustable(min = 0.0, max = 1.0, step = 0.01)
    public double getShoulderPower() {
        return shoulderPower;
    }
    public void setShoulderPower(double power) {
        this.shoulderPower = power;
        if (adjustmentMode) {
            this.shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.shoulder.setTargetPosition(0);
            this.shoulder.setPower(power);
        }
    }

    @Adjustable(min = 0.0, max = 1.0, step = 0.01)
    public double getSlidertPower() {
        return sliderPower;
    }
    public void setSweeperOutPower(double power) {
        this.sliderPower = power;
        if (adjustmentMode) {
            this.shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.shoulder.setTargetPosition(0);
            this.shoulder.setPower(-1 * power);
        }
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
            debug("Slider Adjustment: ON, position: %d / %d, power: %.3f / %.3f", armSlider.getCurrentPosition(), sliderExtended, armSlider.getPower(), sliderPower);
            this.armSlider.setTargetPosition(this.sliderExtended);
            this.armSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.armSlider.setPower(this.sliderPower);
        }
    }

    @Adjustable(min = 0.0, max = 8000.0, step = 1.0)
    public int getSliderDump() {
        return sliderDump;
    }
    public void setSliderDump(int sliderDump) {
        this.sliderDump = sliderDump;
        if (adjustmentMode) {
            debug("Slider Adjustment: ON, position: %d / %d, power: %.3f / %.3f", armSlider.getCurrentPosition(), sliderDump, armSlider.getPower(), sliderPower);
            this.armSlider.setTargetPosition(sliderDump);
            this.armSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.armSlider.setPower(this.sliderPower);
        }
    }

    public int getSliderInitOut() {
        return sliderInitOut;
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
    public void slideIn() { if (armSlider!=null) armSlider.setPower(sliderPower);}
    public void slideOut() { if (armSlider!=null) armSlider.setPower(-sliderPower);}
    public void slideStop() { if (armSlider!=null) armSlider.setPower(0);}

    public void shoulderUp() { if (shoulder!=null) shoulder.setPower(shoulderPower);}
    public void shoulderDown() { if (shoulder!=null) shoulder.setPower(-shoulderPower);}
    public void shoulderStop() { if (shoulder!=null) shoulder.setPower(0);}

    public void configure(Configuration configuration) {
        sweeperServo = configuration.getHardwareMap().servo.get("sv_sweeper");

        gate = new AdjustableServo(GATE_OPEN, GATE_CLOSED).configureLogging(
                logTag + ":boxGate" , logLevel
        );
        gate.configure(configuration.getHardwareMap(), "arm_gate");
        configuration.register(gate);

        shoulder = configuration.getHardwareMap().tryGet(DcMotor.class, "shoulder");
        shoulder.setDirection(DcMotor.Direction.FORWARD);

        armSlider = configuration.getHardwareMap().tryGet(DcMotor.class, "arm_slider");
        armSlider.setDirection(DcMotor.Direction.FORWARD);

        configuration.register(this);
    }

    public void reset(boolean auto) {
        sweeperServo.setPosition(SWEEPER_OFF);
        gate.setPosition(GATE_CLOSED);
        resetMotor(shoulder);
        if (auto) {
            resetMotor(armSlider);
        }
        debug("Reset mininal arm, sweeper: %.2f, gate: %.1f, shoulder: %s / %d, slider: %s / %d",
                sweeperServo.getPosition(), gate.getPosition(),
                shoulder.getMode(), shoulder.getCurrentPosition(),
                armSlider.getMode(), armSlider.getCurrentPosition()
        );
    }


    /**
     * Moves collection box gate to specified position (open or closed)
     * @param open <code>true</code> to open the gate, <code>false</code> to close it
     * @return estimated progress
     */
    public Progress moveGate(boolean open) {
        double target = open ? GATE_OPEN : GATE_CLOSED;
        double adjustment = Math.abs(gate.getPosition() - target);
        debug("moveGate(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from open to closed takes 0.25 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(250 * adjustment);
        gate.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public boolean isGateOpen() {
        return Math.abs(gate.getPosition() - GATE_OPEN) < 0.01;
    }

    public void mineralDumpCombo() {
        final String taskName = "mineralDump";
        if (!TaskManager.isComplete(taskName)) return;

//        TaskManager.add(new Task() {
//            @Override
//            public Progress start() {
//                moveSliderFast(getSliderDump());
//                final Progress boxProgress = moveBox(true);
//                return new Progress() {
//                    @Override
//                    public boolean isDone() {
//                        return boxProgress.isDone() && Math.abs(getSliderCurrent() - getSliderDump()) < 30;
//                    }
//                };
//            }
//        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGate(true);
            }
        }, taskName);
    }

    /**
     * Moves the slider to given position
     * @param position to move slider to
     * @return operation showing whether movement is complete
     */
    public Progress moveSlider(int position) {
        if (position < this.sliderContracted) {
            throw new IllegalArgumentException("Slider position cannot be less than [sliderContracted]");
        }
        if (position > this.sliderExtended) {
            throw new IllegalArgumentException("Slider position cannot be greater than [sliderExtended]");
        }

        this.armSlider.setTargetPosition(position);
        this.armSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.armSlider.setPower(this.sliderPower);
        return new Progress() {
            @Override
            public boolean isDone() {
                return armSlider.isBusy();
            }
        };
    }
    public Progress moveSliderFast(int position) {
        if (position < this.sliderContracted) {
            throw new IllegalArgumentException("Slider position cannot be less than [sliderContracted]");
        }
        if (position > this.sliderExtended) {
            throw new IllegalArgumentException("Slider position cannot be greater than [sliderExtended]");
        }

        this.armSlider.setTargetPosition(position);
        this.armSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.armSlider.setPower(0.9);
        return new Progress() {
            @Override
            public boolean isDone() {
                return armSlider.isBusy();
            }
        };
    }

    public void adjustSlider(boolean extend) {
        this.armSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.armSlider.setPower(this.sliderPower * (extend ? 1 : -1));
    }

    public void stopSlider() {
        this.armSlider.setPower(0);
    }

    public int getSliderCurrent() {
        return this.armSlider.isBusy() ? this.armSlider.getCurrentPosition() : this.armSlider.getTargetPosition();
    }

    public int getSliderTarget() {
        return this.armSlider.getTargetPosition();
    }

    /**
     * Set up telemetry lines for intake metrics
     * Shows encoder values for slider / sweeper and box position
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        line.addData("Sweeper/gate", new Func<String>() {
            @Override
            public String value() {
                return (sweeperServo.getPosition()==0.5 ? "Stop" : "Moving") + "/"
                        + (isGateOpen() ? "Open" : "Closed");
            }
        });
        line.addData("Slide", new Func<String>() {
            @Override
            public String value() {
                int position = armSlider.getTargetPosition();
                String name = "0";
                if (position==sliderDump) name = "Dump";
                if (position==sliderExtended) name = "Ext";
                return String.format("%s:%d", name, armSlider.getCurrentPosition());
            }
        });
        line.addData("shoulder", "%d", new Func<Integer>() {
            @Override
            public Integer value() {
                return shoulder.getCurrentPosition();
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
