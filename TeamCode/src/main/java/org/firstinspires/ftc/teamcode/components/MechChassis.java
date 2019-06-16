package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import static java.lang.Thread.sleep;

/**
 * Swerve chassis consists of 4 wheels with a Servo and DcMotor attached to each.
 * Track (distance between left and right wheels), wheelbase (distance between front and back wheels)
 * and wheel radius are adjustable.
 * Expected hardware configuration is:<br />
 * Servos: servoFrontLeft, servoFrontRight, servoBackLeft, servoBackRight.<br />
 * Motors: motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight.<br />
 * Orientation sensors (optional): imu, imu2
 */
public class MechChassis extends Logger<MechChassis> implements Configurable {

    final private CoreSystem core;

    public enum DriveMode {
        STOP,      // not moving
        STRAIGHT,  // driving in a straight line utilizing orientation sensor to correct itself,
        //  all servos are set to the same position
        ROTATE,    // rotating in place, all servos are set on tangent lines to a circle drawn
        //  through the centers of 4 wheels.
        STEER      // motor power and servos in the direction of movement are controlled by
        //  the driver; opposite servos are in central position
    }

    // distance between the centers of left and right wheels, inches
    private double track = 11.5;
    // distance between the centers of front and back wheels, inches
    private double wheelBase = 10.7;
    // wheel radius, inches
    private double wheelRadius = 2.0;
    // minimum power that should be applied to the wheel motors for robot to start moving
    private double minPower = 0.15;
    // maximum power that should be applied to the wheel motors
    private double maxPower = 0.99;
    private double maxRange = 127; // max range sensor detectable

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    // array contains the same wheel assemblies as above variables
    //  and is convenient to use when actions have to be performed on all 4
    public CombinedOrientationSensor orientationSensor;

    private DriveMode driveMode = DriveMode.STOP;      // current drive mode
    private double targetHeading;     // intended heading for DriveMode.STRAIGHT as reported by orientation sensor
    private double headingDeviation;  // current heading deviation for DriveMode.STRAIGHT as reported by orientation sensor
    private double servoCorrection;   // latest correction applied to leading wheels' servos to correct heading deviation
    private double defaultScale = 0.8;
    private double curHeading = 0;
    private boolean useScalePower = true;//
    private boolean setImuTelemetry = false;//unless debugging, don't set telemetry for imu
    private boolean setRangeSensorTelemetry = false;//unless debugging, don't set telemetry for range sensor

    final double TICKS_PER_CM = 16.86;//number of encoder ticks per cm of driving

    public void enableRangeSensorTelemetry() { // must be call before reset() or setupTelemetry()
        setRangeSensorTelemetry = true;
    }

    public void enableImuTelemetry() {
        setImuTelemetry = true;
    }

    public double getDefaultScale() {
        return defaultScale;
    }

    public void setDefaultScale(double val) {
        defaultScale = val;
    }

    @Adjustable(min = 8.0, max = 18.0, step = 0.02)
    public double getTrack() {
        return track;
    }

    public void setTrack(double track) {
        this.track = track;
    }

    @Adjustable(min = 8.0, max = 18.0, step = 0.02)
    public double getWheelBase() {
        return wheelBase;
    }

    public void setWheelBase(double wheelBase) {
        this.wheelBase = wheelBase;
    }

    @Adjustable(min = 1.0, max = 5.0, step = 0.02)
    public double getWheelRadius() {
        return wheelRadius;
    }

    public void setWheelRadius(double wheelRadius) {
        this.wheelRadius = wheelRadius;
    }

    @Adjustable(min = 0.0, max = 0.4, step = 0.01)
    public double getMinPower() {
        return minPower;
    }

    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }

    @Adjustable(min = 0.2, max = 1.0, step = 0.01)
    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    @Override
    public String getUniqueName() {
        return "chassis";
    }

    /**
     * SwerveChassis constructor
     */
    public MechChassis(CoreSystem core) {
        this.core = core;
    }

    /**
     * Used only for ToboRuckus, old code
     */
    @Deprecated
    public MechChassis() {
        this.core = new CoreSystem(); // Prevents null pointer exception
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    public void configure(Configuration configuration, boolean auto) {
        // set up motors / sensors as wheel assemblies

        motorFL = configuration.getHardwareMap().tryGet(DcMotor.class, "motorFL");
        motorFR = configuration.getHardwareMap().tryGet(DcMotor.class, "motorFR");
        motorBL = configuration.getHardwareMap().tryGet(DcMotor.class, "motorBL");
        motorBR = configuration.getHardwareMap().tryGet(DcMotor.class, "motorBR");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        if (auto || setImuTelemetry) {
            orientationSensor = new CombinedOrientationSensor().configureLogging(logTag + "-sensor", logLevel);
            orientationSensor.configure(configuration.getHardwareMap(), "imu", "imu2");
        }

        // register chassis as configurable component
        configuration.register(this);
    }

    /**
     * move in the vertical direction
     *
     * @param sgn   can be either +1 or -1
     * @param power must be in range [0,1]
     */
    public void yMove(int sgn, double power) {
        motorFL.setPower(sgn * power);
        motorFR.setPower(sgn * power);
        motorBL.setPower(sgn * power);
        motorBR.setPower(sgn * power);
    }

    /**
     * move in the horizontal direction
     *
     * @param sgn   can be either +1 or -1
     * @param power must be in range [0,1]
     */
    public void xMove(int sgn, double power) {
        motorFL.setPower(sgn * power);
        motorFR.setPower(-sgn * power);
        motorBL.setPower(-sgn * power);
        motorBR.setPower(sgn * power);
    }

    /**
     * pivot and turn
     *
     * @param sgn   can be either +1 or -1(+1 for clockwise, -1 for counter clockwise)
     * @param power must be in range [0,1]
     */
    public void turn(int sgn, double power) {
        motorFL.setPower(sgn * power);
        motorFR.setPower(-sgn * power);
        motorBL.setPower(sgn * power);
        motorBR.setPower(-sgn * power);
    }

    /**
     * turning while driving
     *
     * @param power         must be in range [0,1]
     * @param turningFactor int range [-1,+1] (+1 for turning right, -1 for turning left)
     */
    public void carDrive(double power, double turningFactor) {
        if (turningFactor>0){
            turningFactor = 1 - turningFactor;
            motorFL.setPower(power);
            motorFR.setPower(turningFactor * power);
            motorBL.setPower(power);
            motorBR.setPower(turningFactor * power);
        }else {
            turningFactor = 1 + turningFactor;
            motorFL.setPower(turningFactor * power);
            motorFR.setPower(power);
            motorBL.setPower(turningFactor * power);
            motorBR.setPower(power);
        }

    }

    public void stop() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
    public void reset() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        driveMode = DriveMode.STOP;
        targetHeading = 0;
    }


    public enum Direction {
        FRONT, LEFT, RIGHT, BACK;
    }

    public enum Wall {
        LEFT, RIGHT;
    }

    public double getCurHeading() {
        return curHeading;
    }

    /**
     * Scales power according to <code>minPower</code> and <code>maxPower</code> settings
     */
    private double scalePower(double power) {
        double adjustedPower = Math.signum(power) * minPower + power * (maxPower - minPower);
        return Math.abs(adjustedPower) > 1.0 ? Math.signum(adjustedPower) : adjustedPower;
    }

    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        line.addData("Pwr/Scale", new Func<String>() {
            @Override
            public String value() {
                return String.format("%.2f / %.1f", motorFL.getPower(), getDefaultScale());
            }
        });
        if (motorFL != null) {
            line.addData("FL", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorFL.getCurrentPosition();
                }
            });
        }
        if (motorFR != null) {
            line.addData("FR", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorFR.getCurrentPosition();
                }
            });
        }
        if (motorBL != null) {
            line.addData("BL", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorBL.getCurrentPosition();
                }
            });
        }
        if (motorBR != null) {
            line.addData("BR", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorBR.getCurrentPosition();
                }
            });
        }


        //set up imu telemetry
        if (orientationSensor != null && setImuTelemetry) {
            line.addData("imuC", "%.1f", new Func<Double>() {
                @Override
                public Double value() {
                    return orientationSensor.getHeading();
                }
            });
            orientationSensor.setupTelemetry(line);
        }

        telemetry.addLine().addData("M", new Func<String>() {
            @Override
            public String value() {
                return driveMode.name();
            }
        }).addData("Head", new Func<String>() {
            @Override
            public String value() {
                if (driveMode != DriveMode.STRAIGHT) return "N/A";
                return String.format("%+.1f (%+.2f)", targetHeading, headingDeviation);
            }
        }).addData("Adj", new Func<String>() {
            @Override
            public String value() {
                if (driveMode != DriveMode.STRAIGHT) return "N/A";
                return String.format("%+.1f", servoCorrection);
            }
        });
    }

    public void resetOrientation() {
        orientationSensor.reset();
    }

    public boolean hasRollStabalized(int inputIndex, double minDiff) {
        return orientationSensor.hasRollStabalized(inputIndex, minDiff);
    }
}
