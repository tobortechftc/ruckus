package org.firstinspires.ftc.teamcode.hardware.titan;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import java.math.BigDecimal;

/**
 * MineralArm consists of 2 motors (1 arm shoulder, and 1 controlling the slider)
 * and 1 servo controlling the angle of the collection box.<br />
 * Expected hardware configuration is:<br />
 * Servos:  sweeper (continuous servo), gate
 * Motors: shoulder and arm_slider <br />
 */
public class MineralArm extends Logger<MineralArm> implements Configurable {

    final private CoreSystem core;

    public enum ArmMode {INTAKE, VERTICAL_POS, DUMP_SHORT, DUMP_LONG}

    // open and closed positions for the box gate
    // actual servo positions are configured via <code>AdjustableServo</code>
    public static final double GATE_OPEN = 0.0;
    public static final double GATE_PARTIAL_OPEN = 0.3;
    public static final double GATE_CLOSED = 1.0;
    public static final double PUSHER_UP = 1.0;
    public static final double PUSHER_DOWN = 0.0;

    public static final double SWEEPER_OFF = 0.0;
    public static final double SWEEPER_IN = 1.0;
    public static final double SWEEPER_OUT = -1.0;

    public static final int SHOULDER_INIT_POS = 0;
    public static final int SHOULDER_VERTICAL_POS = 1482;
    public static final int SHOULDER_DUMP_SHORT_POS = 777;
    public static final int SHOULDER_DUMP_LONG_POS = 677;
    public static final int SHOULDER_INTAKE_POS = 3524;
    public static final int SHOULDER_MAX_POS = 3541;

    public static final int ARM_SLIDE_MAX_POS = 5050;
    public static final int ARM_SLIDE_DUMP_POS = 5050;
    public static final int ARM_SLIDE_MIN_INTAKE_POS = 0;
    public static final int ARM_SLIDE_INIT_POS = 0;

    private DcMotor shoulder;
    private DcMotor armSlider;
    private CRServo sweeperServo;
    private AdjustableServo gate;
    private AdjustableServo pusher;
    private boolean adjustmentMode = false;
    private AnalogInput potentiometer;

    // slider encoder positions
    private int sliderContracted = 0; // contracted
    private int sliderExtended = ARM_SLIDE_MAX_POS; // fully extended
    private int sliderDump = 300; // position to dump minerals into delivery box
    private int sliderInitOut = 450; // position for initial TeleOp out
    private int sliderAutoPark = 650; // position for Auto Out parking;
    private double sliderPower = 0.8;
    private double shoulderPower = 0.8;
    private int shoulder_init_pos = 0;
    private int shoulder_intake_pos = SHOULDER_INTAKE_POS;
    private int shoulder_dump_short_pos = SHOULDER_DUMP_SHORT_POS;
    private int shoulder_dump_long_pos = SHOULDER_DUMP_LONG_POS;
    private int shoulder_vertical_pos = SHOULDER_VERTICAL_POS;

    /**
     * MineralArm constructor
      */
    public MineralArm(CoreSystem core) {
        this.core = core;
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
            debug("Adjustment: ON, sweeper: %.2f, gate: %.1f, shoulder: %s / %d, slider: %s / %d",
                    sweeperServo.getPower(), gate.getPosition(),
                    shoulder.getMode(), shoulder.getCurrentPosition(),
                    armSlider.getMode(), armSlider.getCurrentPosition()
            );
        } else {
            this.armSlider.setPower(0);
            this.shoulder.setPower(0);
            this.sweeperServo.setPower(SWEEPER_OFF);
            this.gate.setPosition(GATE_CLOSED);
            this.pusherUp();
            debug("Adjustment: OFF, sweeper: %.2f, gate: %.1f, shoulder: %s / %d, slider: %s / %d",
                    sweeperServo.getPower(), gate.getPosition(),
                    shoulder.getMode(), shoulder.getCurrentPosition(),
                    armSlider.getMode(), armSlider.getCurrentPosition()
            );
        }
    }

    public void sweeperIn() {
        this.sweeperServo.setPower(SWEEPER_IN);
    }

    public void sweeperOut() {
        this.sweeperServo.setPower(SWEEPER_OUT);
    }

    public void stopSweeper() {
        this.sweeperServo.setPower(SWEEPER_OFF);
    }

    public void pusherPush(){
       pusher.setPosition(PUSHER_DOWN);
    }

    public void pusherUp(){
        pusher.setPosition(PUSHER_UP);
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

    public void setArmPosition(ArmMode mode) {
        switch(mode) {
            case INTAKE:
                setShoulderPosition(shoulder_intake_pos);
                moveSlider(ARM_SLIDE_MIN_INTAKE_POS);
                break;
            case DUMP_SHORT:
                setShoulderPosition(shoulder_dump_short_pos);
                moveSlider(ARM_SLIDE_DUMP_POS);
                break;
            case DUMP_LONG:
                setShoulderPosition(shoulder_dump_long_pos);
                // moveSlider(ARM_SLIDE_DUMP_POS);
                break;
            case VERTICAL_POS:
                setShoulderPosition(shoulder_vertical_pos);
                moveSlider(ARM_SLIDE_DUMP_POS);
                break;
        }
    }

    public void mineralDumpCombo()  {
        final String taskName = "armShake";
        if (!TaskManager.isComplete(taskName)) return;
        gateClose();
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                int cur_pos = shoulder.getCurrentPosition();
                final Progress armProgress = setShoulderPosition(cur_pos+75);;
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return armProgress.isDone();
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                int cur_pos = shoulder.getCurrentPosition();
                final Progress armProgress = setShoulderPosition(cur_pos-75);;
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return armProgress.isDone();
                    }
                };
            }
        }, taskName);
    }


    public Progress setShoulderPosition(int pos) {
        this.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.shoulder.setTargetPosition(pos);
        this.shoulder.setPower(shoulderPower);
        return new Progress() {
            @Override
            public boolean isDone() {
                return shoulder.isBusy();
            }
        };
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
    public void slideIn(double scale, boolean force) {
        if (armSlider==null)
            return;
        if (!force && armSlider.getCurrentPosition()<0)
            return;
        armSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlider.setPower(-sliderPower*scale);
    }
    public void slideOut(double scale, boolean force) {
        if (armSlider==null) return;
        if (!force && armSlider.getCurrentPosition()>ARM_SLIDE_MAX_POS)
            return;
        armSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlider.setPower(sliderPower*scale);
    }

    public void slideStop() {
        if (armSlider==null) return;
        armSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlider.setPower(0);
    }

    public void shoulderUp(double scale) {
        if (shoulder==null)
            return;
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setPower(shoulderPower*scale);
    }
    public void shoulderDown(double scale, boolean force) {
        if (shoulder==null)
            return;
        if (!force && (shoulder.getCurrentPosition()<0))
            return;
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setPower(-shoulderPower*scale);
    }
    public void shoulderStop() {
        if (shoulder==null)
            return;
        shoulder.setPower(0);}

    public void configure(Configuration configuration) {
        sweeperServo = configuration.getHardwareMap().crservo.get("sv_sweeper");
        sweeperServo.setPower(0);

        gate = new AdjustableServo(GATE_OPEN, GATE_CLOSED).configureLogging(
                logTag + ":armGate", logLevel
        );
        gate.configure(configuration.getHardwareMap(), "arm_gate");
        configuration.register(gate);

        pusher = new AdjustableServo(PUSHER_DOWN, PUSHER_UP).configureLogging(
                logTag + ":pusher", logLevel
        );
        pusher.configure(configuration.getHardwareMap(), "pusher");
        configuration.register(pusher);

        shoulder = configuration.getHardwareMap().tryGet(DcMotor.class, "shoulder");
        shoulder.setDirection(DcMotor.Direction.FORWARD);

        armSlider = configuration.getHardwareMap().tryGet(DcMotor.class, "arm_slider");
        armSlider.setDirection(DcMotor.Direction.REVERSE);

        potentiometer = configuration.getHardwareMap().analogInput.get("pm1");
        coefficients = new BigDecimal[6];
        coefficients[0] = new BigDecimal("0.113543");
        coefficients[1] = new BigDecimal("1.4075");
        coefficients[2] = new BigDecimal("8.87737E-3");
        coefficients[3] = new BigDecimal("1.17293E-4");
        coefficients[4] = new BigDecimal("4.5348E-7");
        coefficients[5] = new BigDecimal("-5.9825E-10");

        configuration.register(this);
    }

    BigDecimal[] coefficients;

    public double getDegree() {
        BigDecimal reading = new BigDecimal(270 * (potentiometer.getVoltage() - 0.001) / (3.35 - 0.001));
        BigDecimal fx = BigDecimal.ZERO;
        for (int i = 5; i >= 0; i--) {
            fx = fx.multiply(reading);
            fx = fx.add(coefficients[i]);
        }
        return fx.doubleValue();
    }

    public void reset(boolean auto) {
        sweeperServo.setPower(SWEEPER_OFF);
        gate.setPosition(GATE_CLOSED);
        resetMotor(shoulder);
        if (auto) {
            resetMotor(armSlider);
        }
        debug("Reset mininal arm, sweeper: %.2f, gate: %.1f, shoulder: %s / %d, slider: %s / %d",
                sweeperServo.getPower(), gate.getPosition(),
                shoulder.getMode(), shoulder.getCurrentPosition(),
                armSlider.getMode(), armSlider.getCurrentPosition()
        );
    }


    /**
     * Moves collection arm gate to specified position (open or closed)
     *
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

    public void gateOpen() { gate.setPosition(GATE_OPEN);}
    public void gatePartialOpen() { gate.setPosition(GATE_PARTIAL_OPEN);}
    public void gateClose() { gate.setPosition(GATE_CLOSED);}
    public void gateAuto() {
        if (isGateOpen()) {
            // gateClose();
            mineralDumpCombo();
        } else {
            gateOpen();
        }
    }


    public boolean isGateOpen() {
        return Math.abs(gate.getPosition() - GATE_OPEN) < 0.01;
    }

    /**
     * Moves the slider to given position
     *
     * @param position to move slider to
     * @return operation showing whether movement is complete
     */
    public Progress moveSlider(int position) {
        //if (position < this.sliderContracted) {
        //    throw new IllegalArgumentException("Slider position cannot be less than [sliderContracted]");
        //}
        //if (position > this.sliderExtended) {
        //    throw new IllegalArgumentException("Slider position cannot be greater than [sliderExtended]");
        //}

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
                return (sweeperServo.getPower() == 0.0 ? "Stop" : "Moving") + "/"
                        + (isGateOpen() ? "Open" : "Closed");
            }
        });
        line.addData("Slide", new Func<String>() {
            @Override
            public String value() {
                int position = armSlider.getTargetPosition();
                String name = "0";
                if (position == sliderDump) name = "Dump";
                if (position == sliderExtended) name = "Ext";
                return String.format("%s:%d", name, armSlider.getCurrentPosition());
            }
        });
        line.addData("shoulder", "%d", new Func<Integer>() {
            @Override
            public Integer value() {
                return shoulder.getCurrentPosition();
            }
        });
        line.addData("degree", "%f", new Func<Double>() {
            @Override
            public Double value() {
                return getDegree();
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
