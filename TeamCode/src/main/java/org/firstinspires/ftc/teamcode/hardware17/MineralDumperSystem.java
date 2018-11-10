package org.firstinspires.ftc.teamcode.hardware17;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Put brief class description here...
 */
public class MineralDumperSystem {
//    public boolean use_verbose = false;
    public boolean use_dumper = false;
    public boolean use_dumper_gate = true;
    boolean gg_slider_encoder_ok = false; // used by glyph lifter

    final static double SV_DUMPER_INIT = 0.6822;
    final static double SV_DUMPER_DOWN = 0.6822;
    public final static double SV_DUMPER_LIFT = 0.599;
    final static double SV_DUMPER_HALF_UP = 0.551;
    public final static double SV_DUMPER_UP = 0.204;
    final static double SV_DUMPER_DUMP = 0.214;
    final static double SV_DUMPER_GATE_INIT = 0.25;
    final static double SV_DUMPER_GATE_UP = 0.25;
    final static double SV_DUMPER_GATE_DOWN = 0.62;

    final static int LIFT_INIT_COUNT = 30;
    final static int LIFT_MAX_COUNT = 2470+LIFT_INIT_COUNT;
    final static int GG_SLIDE_INIT = 10;
    final static double GG_SLIDE_UP_POWER = 1.0;
    final static double GG_SLIDE_DOWN_POWER = -0.9;
    double intakeRatio = 1.0;

    int init_gg_slider_pos = GG_SLIDE_INIT;
    int target_gg_slider_pos = 0;
    int target_relic_slider_pos = 0;

    public DcMotor mt_lift = null;
    public Servo sv_dumper = null;
    public Servo sv_dumper_gate = null;
    public CRServo sv_bar_wheel = null;

    // Central core of robot
    CoreSystem core;
    ElapsedTime runtime;

    private TaintedAccess taintedAccess;
    void setTaintedAccess(TaintedAccess taintedAccess) {
        this.taintedAccess = taintedAccess;
    }

    MineralDumperSystem(CoreSystem core) {
        this.core = core;
    }

    public void enable(boolean isAuto) {
        if (isAuto) {
            use_dumper = true;
            use_dumper_gate = true;
        } else {
            use_dumper = true;
            use_dumper_gate = true;
        }
    }

    public void disable() {
        use_dumper = false;
        use_dumper_gate = false;
    }

    void init(HardwareMap hwMap) {
        if (use_dumper) {
            sv_dumper = hwMap.servo.get("sv_dumper");
            sv_dumper.setPosition(SV_DUMPER_INIT);
            {
                if (use_dumper_gate) {
                    sv_dumper_gate = hwMap.servo.get("sv_dumper_gate");
                    sv_dumper_gate.setPosition(SV_DUMPER_GATE_INIT);
                }
                // sv_bar_wheel = hwMap.crservo.get("sv_bar_wheel");
                // sv_bar_wheel.setPower(0);
            }
            mt_lift = hwMap.dcMotor.get("mtLift");
            mt_lift.setDirection(DcMotor.Direction.REVERSE);
            mt_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mt_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mt_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void dumper_vertical() {
        if (!use_dumper || sv_dumper==null)
            return;
        sv_dumper.setPosition(SV_DUMPER_UP);
        if (use_dumper_gate) {
            dumperGateDown();
        }
    }

    public void dumper_up() {
        if (!use_dumper || sv_dumper==null)
            return;
        double pos = sv_dumper.getPosition();
        if (Math.abs(pos-SV_DUMPER_DOWN)<0.05)
            sv_dumper.setPosition(SV_DUMPER_HALF_UP);
        else if (Math.abs(pos-SV_DUMPER_HALF_UP)<0.05) {
            sv_dumper.setPosition(SV_DUMPER_UP);
            if (use_dumper_gate) {
                dumperGateDown();
            }
        } else {
            sv_dumper.setPosition(SV_DUMPER_DUMP);
            if (use_dumper_gate) {
                dumperGateDown();
            }
        }
    }

    public void dumper_shake() {
        if (!use_dumper || sv_dumper==null)
            return;
        double pos = sv_dumper.getPosition();
        if ((pos-0.1)>0) {
            sv_dumper.setPosition(pos - 0.1);
            core.yield_for(.2);
        }
        sv_dumper.setPosition(pos);
    }

    public void dumper_down(boolean raiseGate)
    {
        if (!use_dumper || sv_dumper==null)
            return;
        sv_dumper.setPosition(SV_DUMPER_DOWN);
        if (use_dumper_gate && raiseGate) {
            dumperGateUp();
        }
    }

    public void lift_back_init() { // back to initial position
        target_gg_slider_pos = init_gg_slider_pos;
        slide_to_target(GG_SLIDE_DOWN_POWER);
        dumper_down(true);
    }

    public void lift_up(boolean force) {
        double power = 1.0;
        // never exceed GG_SLIDE_MAX_COUNT
        int cur_pos = mt_lift.getCurrentPosition();
        if ((cur_pos>LIFT_MAX_COUNT) && !force) {
            power = 0.0;
        }
        if (sv_dumper.getPosition()>SV_DUMPER_LIFT) {
            sv_dumper.setPosition(SV_DUMPER_LIFT);
        }
        mt_lift.setPower(power);
    }

    public void lift_down(boolean force) {
        double power = -0.95;
        // never lower than 0
        int cur_pos = mt_lift.getCurrentPosition();
        if ((cur_pos<=LIFT_INIT_COUNT) && !force) {
            power = 0.0;
            dumper_down(true);
        }
        mt_lift.setPower(power);
    }

    public void lift_up_and_down(boolean force) {
        lift_up(force);
        core.yield_for(.3);
        lift_stop();
        core.yield_for(.15);
        lift_down(force);
        core.yield_for(.15);
        lift_stop();
    }

    public void lift_stop() {
        mt_lift.setPower(0.0);
    }

    public void show_telemetry(Telemetry telemetry) {
        // I don't know what dumper CPU time is -ND
        //telemetry.addData("0: initialize dumper CPU time =?", "%3.2f sec", core.run_seconds());
        telemetry.addData("8. gg-rot pwr/cur/tar = ", "%3.2f/%d/%d", mt_lift.getPower(), mt_lift.getCurrentPosition(), target_gg_slider_pos);
    }
    public void slide_to_target(double power) {
        if (!gg_slider_encoder_ok)
            return;

        if (power<0) power=-1.0*power;

        // Why?
        DcMotor mt = mt_lift;

        mt.setTargetPosition(target_gg_slider_pos);
        mt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        mt.setPower(Math.abs(power));
        while (mt.isBusy() && (runtime.seconds() < 3))
            core.yield();
        mt.setPower(Math.abs(power / 2.0));
        while (mt.isBusy() && (runtime.seconds() < 1))
            core.yield();
        mt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mt.setPower(0);
    }

    void dumperGateUp() {
        if (!use_dumper || sv_dumper_gate==null)
            return;
        sv_dumper_gate.setPosition(SV_DUMPER_GATE_UP);
    }

    void dumperGateDown() {
        if (!use_dumper || sv_dumper_gate==null)
            return;
        sv_dumper_gate.setPosition(SV_DUMPER_GATE_DOWN);
    }

}