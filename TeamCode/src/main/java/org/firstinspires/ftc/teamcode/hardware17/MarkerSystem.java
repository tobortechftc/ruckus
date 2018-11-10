package org.firstinspires.ftc.teamcode.hardware17;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Put brief class description here...
 */
public class MarkerSystem {
    public boolean use_verbose = false;
    public boolean use_color_sensor = false;
    public boolean use_arm = false;

    final static int RED_BALL_MIN = -600;
    final static int RED_BALL_MAX = -15;
    final static int BLUE_BALL_MIN = 12;
    final static int BLUE_BALL_MAX = 600;

    final static double SV_SHOULDER_INIT = 0.47;
    final static double SV_SHOULDER_DOWN = 0.46;
    final static double SV_SHOULDER_LEFT_3 = 0.6294;
    final static double SV_SHOULDER_LEFT_2 = 0.585;
    final static double SV_SHOULDER_LEFT_1 = 0.535;
    final static double SV_SHOULDER_RIGHT_3 = 0.30;
    final static double SV_SHOULDER_RIGHT_2 = 0.40;
    final static double SV_SHOULDER_RIGHT_1 = 0.45;

    final static double SV_ELBOW_UP = 0.997;
    final static double SV_ELBOW_DOWN = 0.44;
    final static double SV_ELBOW_DOWN_HIT = 0.43;

    final static double SV_RIGHT_ARM_UP = 0.11;
    final static double SV_RIGHT_ARM_DOWN = 0.73;
    final static double SV_RIGHT_ARM_UP_NB = 0.02;
    final static double SV_RIGHT_ARM_DOWN_NB = 0.43;
    final static double SV_LEFT_ARM_UP_NB = 0.943;
    final static double SV_LEFT_ARM_DOWN_NB = 0.359;
    final static double SV_FRONT_ARM_IN = 0.83;
    final static double SV_FRONT_ARM_OUT = 0.43;
    public final static double SV_JKICKER_UP = 0.47;
    final static double SV_JKICKER_RIGHT1 = 0.57;
    final static double SV_JKICKER_RIGHT2 = 0.65;
    final static double SV_JKICKER_RIGHT = 0.85;
    final static double SV_JKICKER_LEFT1 = 0.37;
    final static double SV_JKICKER_LEFT2 = 0.27;
    final static double SV_JKICKER_LEFT = 0.1;
    final static double SV_JKICKER_INIT = 0.82;

    double blue = 0; // [convert to local]
    double red = 0; // [convert to local ]

    public Servo sv_shoulder = null;
    public Servo sv_elbow = null;
    public Servo sv_left_arm = null;
    public Servo sv_right_arm = null;
    public Servo sv_jkicker = null;

    public ColorSensor l_colorSensor = null;
    public ColorSensor r_colorSensor = null;

    // Central core of robot
    CoreSystem core;
    ElapsedTime runtime;

    private TaintedAccess taintedAccess;
    void setTaintedAccess(TaintedAccess taintedAccess) {
        this.taintedAccess = taintedAccess;
    }

    MarkerSystem(CoreSystem core) {
        this.core = core;
    }

    public void enable(boolean isAuto) {
        if (isAuto) {
            use_color_sensor = true;
            use_arm = true;
        } else {
            use_color_sensor = false;
            use_arm = true;
        }
    }

    public void disable() {
        use_color_sensor = false;
        use_arm = false;
    }

    void init(HardwareMap hwMap) {
        if (use_color_sensor) {
            r_colorSensor = hwMap.get(ColorSensor.class, "colorRight");
            r_colorSensor.enableLed(true);

        }
        if (use_arm) {
                    sv_right_arm = hwMap.servo.get("sv_right_arm");
                    sv_right_arm.setPosition(SV_RIGHT_ARM_UP_NB);
                    // sv_jkicker = hwMap.servo.get("sv_jkicker");
                    // sv_jkicker.setPosition(SV_JKICKER_INIT);

        }
    }

    public void stop () {
        if (l_colorSensor!=null) {
            l_colorSensor.enableLed(false);
            l_colorSensor.close();
        }
        if (r_colorSensor!=null) {
            r_colorSensor.enableLed(false);
            r_colorSensor.close();
        }
    }

    public double calcDelta(boolean isBlueAlliance) throws InterruptedException {
        if(!use_color_sensor) {
            return 0;
        }
        blue = r_colorSensor.blue();
        red = r_colorSensor.red();
        return (blue - red);
    }

    public SwerveUtilLOP.TeamColor checkBallColor(boolean isBlueAlliance) throws InterruptedException {
        boolean isBlueBall = false;
        boolean isRedBall = false;
        runtime.reset();
        double d = calcDelta(isBlueAlliance);
        SwerveUtilLOP.TeamColor result = SwerveUtilLOP.TeamColor.UNKNOWN;

        while (!isBlueBall && !isRedBall && (runtime.seconds()<1.5)) {
            if ((d >= BLUE_BALL_MIN) && (d <= BLUE_BALL_MAX)) {
                isBlueBall = true;
            } else {
                isBlueBall = false;
            }
            if (d >= RED_BALL_MIN && d <= RED_BALL_MAX) {
                isRedBall = true;
            } else {
                isRedBall = false;
            }

            d = calcDelta(isBlueAlliance);
        }
//        telemetry.addData("delta/isBlueBall/isRedBall=", "%3.1f/%s/%s",d,isBlueBall,isRedBall);
//        telemetry.update();
        if (isBlueBall && isRedBall) {
            result = SwerveUtilLOP.TeamColor.UNKNOWN;
        } else if (isBlueBall) {
            result = SwerveUtilLOP.TeamColor.BLUE;
        }
        else if (isRedBall) {
            result = SwerveUtilLOP.TeamColor.RED;
        }
        else {
            result = SwerveUtilLOP.TeamColor.UNKNOWN;
        }
        return result;
    }

    public void l_arm_up() {
        {
            if (sv_elbow!=null)
                sv_elbow.setPosition(SV_ELBOW_UP);
            core.yield_for(.2);
            if (sv_shoulder!=null)
                sv_shoulder.setPosition(SV_SHOULDER_INIT);
        }
    }

    public void l_arm_down() {
        {
            if (sv_elbow!=null)
                sv_elbow.setPosition(SV_ELBOW_DOWN);
            if (sv_shoulder!=null)
                sv_shoulder.setPosition(SV_SHOULDER_DOWN);
        }
    }

    public void r_arm_up() {
        if (sv_jkicker!=null) {
            jkick_up();
        }
        if (sv_right_arm!=null) {
            sv_right_arm.setPosition(SV_RIGHT_ARM_UP_NB);
        }
        core.yield_for(.2);

    }

    public void r_arm_down() {
        if (sv_right_arm!=null) {
            sv_right_arm.setPosition(SV_RIGHT_ARM_DOWN_NB);
        }
        core.yield_for(.2);
    }

    public void jkick_right() {
        if (sv_jkicker==null) return;
        sv_jkicker.setPosition(SV_JKICKER_RIGHT1);
        core.yield_for(.1);
        sv_jkicker.setPosition(SV_JKICKER_RIGHT2);
        core.yield_for(.15);
        sv_jkicker.setPosition(SV_JKICKER_RIGHT);
        core.yield_for(.25);
    }

    public void jkick_left() {
        if (sv_jkicker==null) return;
        sv_jkicker.setPosition(SV_JKICKER_LEFT1);
        core.yield_for(.1);
        sv_jkicker.setPosition(SV_JKICKER_LEFT2);
        core.yield_for(.15);
        sv_jkicker.setPosition(SV_JKICKER_LEFT);
        core.yield_for(.25);
    }

    void jkick_up() {
        if (sv_jkicker==null) return;
        sv_jkicker.setPosition(SV_JKICKER_UP);
        core.yield_for(.3);
    }

    public void arm_left() {
        if (sv_elbow!=null)
            sv_elbow.setPosition(SV_ELBOW_DOWN_HIT);
        sv_shoulder.setPosition(SV_SHOULDER_LEFT_1);
        core.yield_for(.5);
        if (sv_shoulder!=null)
            sv_shoulder.setPosition(SV_SHOULDER_LEFT_2);
        core.yield_for(.5);
        if (sv_shoulder!=null)
            sv_shoulder.setPosition(SV_SHOULDER_LEFT_3);
    }

    public void arm_right() {
        if (sv_elbow!=null)
            sv_elbow.setPosition(SV_ELBOW_DOWN_HIT);
        if (sv_shoulder!=null)
            sv_shoulder.setPosition(SV_SHOULDER_RIGHT_1);
        core.yield_for(.5);
        if (sv_shoulder!=null)
            sv_shoulder.setPosition(SV_SHOULDER_RIGHT_2);
        core.yield_for(.5);
        if (sv_shoulder!=null)
            sv_shoulder.setPosition(SV_SHOULDER_RIGHT_3);
    }

    public void show_telemetry(Telemetry telemetry) {
        telemetry.addData("marker servo =", sv_shoulder.getPosition());
    }

}
