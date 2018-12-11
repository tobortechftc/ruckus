package org.firstinspires.ftc.teamcode.hardware.ruckus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

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
    private DigitalChannel liftTouch;
    private double gateClosePos = 0.33;
    private double gateOpenPos = 0.8;
    private double armInitPos = 0.02;
    private double armDownPos = 0.06;
    private double armSafePos = 0.08;
    private double armDumpPos = 0.82; // actual dump position
    private double armUpPos = 0.92;   // max arm position
    private double liftPower = .90;
    private double liftDownPower = .40;
    private boolean gateIsOpened = false;
    private final int MAX_LIFT_POS = 1120; // old small spool = 4100;
    private final int AUTO_LIFT_POS = 1090; // old small spool = 4000;
    private final int LIFT_COUNT_PER_INCH = 410;

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
        armInit();
    }

    public void configure(Configuration configuration) {
        // set up motors / sensors as wheel assemblies
        lift = configuration.getHardwareMap().dcMotor.get("lift_slider");
        lift.setPower(0);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dumperGate = configuration.getHardwareMap().servo.get("sv_hp_gate");
        dumperArm = configuration.getHardwareMap().servo.get("sv_hp_dump");
        liftTouch = configuration.getHardwareMap().get(DigitalChannel.class, "lift_touch");
        // set the digital channel to input.
        liftTouch.setMode(DigitalChannel.Mode.INPUT);
        // register delivery as configurable component
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
    public void liftUp(boolean force) {
        int cur_pos = lift.getCurrentPosition();
        if ((cur_pos>MAX_LIFT_POS && force==false) || (liftTouch!=null && liftTouch.getState()==false)) {
            lift.setPower(0);
        } else {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(liftPower);
        }
    }
    public void liftDown(boolean force){
        int cur_pos = lift.getCurrentPosition();
        if (cur_pos<=0 && force==false) {
            lift.setPower(0);
        } else {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(-1 * liftDownPower);
        }
    }

    public Progress liftAuto(boolean up) {
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(up ? AUTO_LIFT_POS : 20);
        if (up==true && liftTouch!=null && liftTouch.getState()==false) {
            // hardware limit hit
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(0);
            return new Progress() {
                @Override
                public boolean isDone() {
                    return true;
                }};
        } else {
            lift.setPower(liftPower);
        }
        return new Progress() {
            @Override
            public boolean isDone() {
                return !lift.isBusy();
            }
        };
    }

    public void liftStop(){
        lift.setPower(0);
    }

    private Progress moveArm(double position) {
        double adjustment = Math.abs(position - dumperArm.getPosition());
        dumperArm.setPosition(position);
        // 3.3ms per degree of rotation
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 800);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public Progress armUp() {
        return moveArm(armUpPos);
    }
    public Progress armInit() {
        return moveArm(armInitPos);
    }
    public Progress armDown() {
        return moveArm(armDownPos);
    }
    public Progress armDump() {
        return moveArm(armDumpPos);
    }
    public Progress armSafeLift() {
        return moveArm(armSafePos);
    }
    public void armDownInc() {
        double cur_pos = dumperArm.getPosition();
        double tar_pos = armDownPos;
        if (cur_pos>armDownPos+0.03) {
            tar_pos = cur_pos - 0.03;
        }
        dumperArm.setPosition(tar_pos);
    }
    public void armUpInc() {
        double cur_pos = dumperArm.getPosition();
        double tar_pos = armUpPos;
        if (cur_pos<armUpPos-0.03) {
            tar_pos = cur_pos + 0.03;
        }
        dumperArm.setPosition(tar_pos);
    }
    public void armStop() {
        double cur_pos = dumperArm.getPosition();
        dumperArm.setPosition(cur_pos);
    }

    public void deliveryCombo(final MineralIntake sliderControl) {
        final String taskName = "deliveryCombo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                gateClose();
                final int safePosition = (sliderControl.getSliderInitOut() + sliderControl.getSliderExtended()) / 2 + 200;
                sliderControl.moveSlider(safePosition);
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return Math.abs(sliderControl.getSliderCurrent() - safePosition) < 5;
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return armSafeLift();
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return liftAuto(true);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return armDump();
            }
        }, taskName);
    }

    public void returnCombo() {
        final String taskName = "returnCombo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return armSafeLift();
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return liftAuto(false);
            }
        }, taskName);
    }

    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     *  drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     *  and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        if (lift!=null)
           line.addData("Lift", "enc=%d", new Func<Integer>() {
               @Override
               public Integer value() {
                   return lift.getCurrentPosition();
               }});

        if(dumperGate!=null){
            line.addData("Gate", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return dumperGate.getPosition();
                }});
        }
        if(dumperArm!=null){
            line.addData("Arm", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return dumperArm.getPosition();
                }});
        }
        if (liftTouch!=null) {
            line.addData("Touch", "state=%s", new Func<String>() {
                @Override
                public String value() {
                    return (liftTouch.getState()?"not-pressed":"pressed");
                }});
        }
    }

}
