package org.firstinspires.ftc.teamcode.hardware17;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Put brief class description here...
 */
public class MineralIntakeSystem {
    public boolean use_intake = false;

    final static double svIntakeGate_INIT = 0.844;
    final static double svIntakeGate_UP = 0.61;
    public final static double svIntakeGate_MID = 0.5;
    final static double svIntakeGate_DOWN = 0.217;
    final static double SV_DUMPER_GATE_INIT = 0.25;
    final static double SV_DUMPER_GATE_UP = 0.25;
    final static double SV_DUMPER_GATE_DOWN = 0.62;

    public DcMotor mtIntakeLeft = null;
    public DcMotor mtIntakeRight = null;
    public Servo svIntakeGate = null;
    public CRServo svBarWheel = null;
    public Servo sv_intake_gate = null;

    public double intakeRatio = 1.0;

    // Central core of robot
    CoreSystem core;

    private TaintedAccess taintedAccess;
    void setTaintedAccess(TaintedAccess taintedAccess) {
        this.taintedAccess = taintedAccess;
    }

    MineralIntakeSystem(CoreSystem core) {
        this.core = core;
    }

    public void enable(boolean isAuto) {
        if (isAuto) {
            use_intake = true;
        } else {
            use_intake = true;
        }
    }

    public void disable() {
        use_intake = false;
    }

    void init(HardwareMap hwMap) {
        if (use_intake) {
            mtIntakeLeft = hwMap.dcMotor.get("mtIntakeLeft");
            mtIntakeLeft.setDirection(DcMotor.Direction.REVERSE);
            mtIntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtIntakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            svIntakeGate = hwMap.servo.get("svIntakeGate");
            svIntakeGate.setPosition(svIntakeGate_INIT);

            mtIntakeRight = hwMap.dcMotor.get("mtIntakeRight");
            // mt_glyph_slider.setDirection(DcMotor.Direction.REVERSE);
            mtIntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtIntakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void correctGlyph(boolean leadClockwise) {
        if (!use_intake)
            return;
        if(leadClockwise) {
            intakeTurn(true);
            core.yield_for(.15);
            intakeTurn(false);
            core.yield_for(.15);
            intakeIn();
            core.yield_for(.3);
        }
        else{
            intakeTurn(false);
            core.yield_for(.15);
            intakeTurn(true);
            core.yield_for(.15);
            intakeIn();
            core.yield_for(.3);
        }
    }
    public void intakeGateInit() {
        if (!use_intake||svIntakeGate==null)
            return;
        svIntakeGate.setPosition(svIntakeGate_INIT);
    }

    public void intakeGateUp() {
        if (!use_intake || svIntakeGate==null)
            return;
        double pos = svIntakeGate.getPosition();
        if (Math.abs(pos-svIntakeGate_DOWN)<0.1)
            svIntakeGate.setPosition(svIntakeGate_MID);
        else if (Math.abs(pos-svIntakeGate_MID)<0.1)
            svIntakeGate.setPosition(svIntakeGate_UP);
        else
            svIntakeGate.setPosition(svIntakeGate_INIT);
    }

    public void intakeGateDown() {
        if (!use_intake || svIntakeGate==null)
            return;
        //if (swerve.use_newbot_v2 && robot.relicReachSystem.use_relic_grabber)
        //    robot.relicReachSystem.relic_arm_up();
        double pos = svIntakeGate.getPosition();
        if (Math.abs(pos-svIntakeGate_INIT)<0.1)
            svIntakeGate.setPosition(svIntakeGate_UP);
        else if (Math.abs(pos-svIntakeGate_UP)<0.1)
            svIntakeGate.setPosition(svIntakeGate_MID);
        else
            svIntakeGate.setPosition(svIntakeGate_DOWN);
    }

    public void intakeBarWheelIn() {
        if (svBarWheel==null)
            return;
        svBarWheel.setPower(0.8);
    }

    public void intakeBarWheelOut() {
        if (svBarWheel==null)
            return;
        svBarWheel.setPower(-0.8);
    }

    public void intakeBarWheelStop() {
        if (svBarWheel==null)
            return;
        svBarWheel.setPower(0);
    }

    public void intakeIn() {
        if (!use_intake)
            return;
//        if (sv_dumper!=null && robot.sv_dumper.getRawPosition()<0.63) {
//            return;
//        }
        mtIntakeLeft.setPower(intakeRatio);
        mtIntakeRight.setPower(intakeRatio);
        intakeBarWheelIn();
    }

    public void intakeTurn(boolean clockwise) {
        if (!use_intake)
            return;
        if (clockwise) {
            mtIntakeLeft.setPower(-intakeRatio / 2.0);
            mtIntakeRight.setPower(intakeRatio);
        } else {
            mtIntakeLeft.setPower(intakeRatio);
            mtIntakeRight.setPower(-intakeRatio / 2);
        }
    }

    public void intakeOut() {
        if (!use_intake)
            return;
        mtIntakeLeft.setPower(-1.0*intakeRatio);
        mtIntakeRight.setPower(-1.0*intakeRatio);
        intakeBarWheelOut();
    }

    public void intakeStop() {
        if (!use_intake)
            return;
        mtIntakeLeft.setPower(0);
        mtIntakeRight.setPower(0);
        intakeBarWheelStop();
    }
    public void show_telemetry(Telemetry telemetry) {
        // filler , not needed
        telemetry.addData("mtIntakeRight =", mtIntakeLeft.getPower());
    }
}
