package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.opencv.core.Mat;

import java.util.Arrays;

/**
 * Swerve chassis consists of 4 wheels with a Servo and DcMotor attached to each.
 * Track (distance between left and right wheels), wheelbase (distance between front and back wheels)
 * and wheel radius are adjustable.
 * Expected hardware configuration is:<br />
 * Servos: servoFrontLeft, servoFrontRight, servoBackLeft, servoBackRight.<br />
 * Motors: motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight.<br />
 * Orientation sensors (optional): imu, imu2
 */
public class SwerveChassis extends Logger<SwerveChassis> implements Configurable {

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
    private double maxPower = 0.5;

    private WheelAssembly frontLeft;
    private WheelAssembly frontRight;
    private WheelAssembly backLeft;
    private WheelAssembly backRight;
    // array contains the same wheel assemblies as above variables
    //  and is convenient to use when actions have to be performed on all 4
    private WheelAssembly[] wheels = new WheelAssembly[4];
    private CombinedOrientationSensor orientationSensor;

    private DistanceSensor frontRangeSensor;
    private DistanceSensor leftRangeSensor;

    private DriveMode driveMode = DriveMode.STOP;      // current drive mode
    private double targetHeading;     // intended heading for DriveMode.STRAIGHT as reported by orientation sensor
    private double headingDeviation;  // current heading deviation for DriveMode.STRAIGHT as reported by orientation sensor
    private double servoCorrection;   // latest correction applied to leading wheels' servos to correct heading deviation


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

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    public void configure(Configuration configuration) {
        // set up motors / sensors as wheel assemblies
        wheels[0] = frontLeft = new WheelAssembly(
                configuration, "FrontLeft", DcMotor.Direction.FORWARD
        );
        wheels[1] = frontRight = new WheelAssembly(
                configuration, "FrontRight", DcMotor.Direction.REVERSE
        );
        wheels[2] = backLeft = new WheelAssembly(
                configuration, "BackLeft", DcMotor.Direction.FORWARD
        );
        wheels[3] = backRight = new WheelAssembly(
                configuration, "BackRight", DcMotor.Direction.REVERSE
        );

        orientationSensor = new CombinedOrientationSensor().configureLogging(logTag + "-sensor", logLevel);
        orientationSensor.configure(configuration.getHardwareMap(), "imu", "imu2");

        leftRangeSensor = configuration.getHardwareMap().get(DistanceSensor.class, "left_range");
        frontRangeSensor = configuration.getHardwareMap().get(DistanceSensor.class, "front_range");

        // register chassis as configurable component
        configuration.register(this);
    }

    public double distanceToLeft(){
        return leftRangeSensor.getDistance(DistanceUnit.CM);
    }

    public double distanceToFront(){
        return frontRangeSensor.getDistance(DistanceUnit.CM);
    }

    public void reset() {
        for (WheelAssembly wheel : wheels) wheel.reset(true);
        driveMode = DriveMode.STOP;
        targetHeading = 0;
    }

    /**
     * Drive in a straight line and maintain heading via IMU
     *
     * @param power   - -1 to 1
     * @param heading - -90 to 90; relative to current robot orientation
     */
    public void driveStraight(double power, double heading) throws InterruptedException {
        debug("driveStraight(pwr: %.3f, head: %.1f)", power, heading);
        if (power < -1 || power > 1) {
            throw new IllegalArgumentException("Power must be between -1 and 1");
        }
        if (heading < -90 || heading > 90) {
            throw new IllegalArgumentException("Heading must be between -90 and 90");
        }

        if (power == 0) {
            driveMode = DriveMode.STOP;
            targetHeading = 0;
            headingDeviation = 0;
            servoCorrection = 0;
            for (WheelAssembly wheel : wheels) wheel.motor.setPower(0);
            orientationSensor.enableCorrections(false);
            return;
        }

        if (driveMode != DriveMode.STRAIGHT) {
            if (driveMode != DriveMode.STOP) {
                // reset motors if they're moving; servos are adjusted below
                for (WheelAssembly wheel : wheels) wheel.reset(false);
            }
            driveMode = DriveMode.STRAIGHT;

            for (WheelAssembly wheel : wheels)
                wheel.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double[] newServoPositions = new double[4];
            Arrays.fill(newServoPositions, heading);
            changeServoPositions(newServoPositions);

            orientationSensor.enableCorrections(true);
            targetHeading = orientationSensor.getHeading();
        } else {
            // check and correct heading as needed
            double sensorHeading = orientationSensor.getHeading();
            headingDeviation = targetHeading - sensorHeading;
            debug("driveStraight(): target=%+.2f, sensor=%+.2f, adjustment=%+.2f)",
                    targetHeading, sensorHeading, headingDeviation);
            if (Math.abs(headingDeviation) > 0.5) {
                servoCorrection = headingDeviation / 2;
                if (power < 0) {
                    backLeft.servo.adjustPosition(servoCorrection);
                    backRight.servo.adjustPosition(servoCorrection);
                } else {
                    frontLeft.servo.adjustPosition(servoCorrection);
                    frontRight.servo.adjustPosition(servoCorrection);
                }
            } else {
                servoCorrection = 0;
                if (power < 0) {
                    backLeft.servo.setPosition(frontLeft.servo.getPosition());
                    backRight.servo.setPosition(frontRight.servo.getPosition());
                } else {
                    frontLeft.servo.setPosition(backLeft.servo.getPosition());
                    frontRight.servo.setPosition(backRight.servo.getPosition());
                }
            }
        }
        for (WheelAssembly wheel : wheels) wheel.motor.setPower(scalePower(power));
    }

    double TICKS_PER_CM = 16.04;

    //using the indicated absolute power to drive a certain distance at a certain heading
    public void driveStraightAuto(double power, double cm, double heading) throws InterruptedException {
        debug("driveStraight(pwr: %.3f, head: %.1f)", power, heading);
        if (power < 0 || power > 1) {
            throw new IllegalArgumentException("Power must be between 0 and 1");
        }
        if (heading < -90 || heading > 90) {
            throw new IllegalArgumentException("Heading must be between -90 and 90");
        }

        double distance = TICKS_PER_CM * cm;

        if (power == 0) {
            driveMode = DriveMode.STOP;
            targetHeading = 0;
            headingDeviation = 0;
            servoCorrection = 0;
            for (WheelAssembly wheel : wheels) wheel.motor.setPower(0);
            orientationSensor.enableCorrections(false);
            return;
        }

        if (distance<0){
            power=-power;
            distance=-distance;
        }

        //motor settings
        driveMode = DriveMode.STRAIGHT;
        int[] startingCount = new int[4];
        for (int i = 0; i < 4; i++) {
            wheels[i].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheels[i].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            startingCount[i] = wheels[i].motor.getCurrentPosition();
        }

        //servo settings
        double[] newServoPositions = new double[4];
        Arrays.fill(newServoPositions, heading);
        changeServoPositions(newServoPositions);

        //imu initialization
        orientationSensor.enableCorrections(true);
        targetHeading = orientationSensor.getHeading();

        //start powering
        for (WheelAssembly wheel : wheels) wheel.motor.setPower(power);

        //waiting loop
        while (true) {
            // check and correct heading as needed
            double sensorHeading = orientationSensor.getHeading();
            headingDeviation = targetHeading - sensorHeading;
            debug("driveStraight(): target=%+.2f, sensor=%+.2f, adjustment=%+.2f)",
                    targetHeading, sensorHeading, headingDeviation);
            if (Math.abs(headingDeviation) > 0.5) {
                servoCorrection = headingDeviation / 2;
                if (power < 0) {
                    backLeft.servo.adjustPosition(servoCorrection);
                    backRight.servo.adjustPosition(servoCorrection);
                } else {
                    frontLeft.servo.adjustPosition(servoCorrection);
                    frontRight.servo.adjustPosition(servoCorrection);
                }
            } else {
                servoCorrection = 0;
                if (power < 0) {
                    backLeft.servo.setPosition(frontLeft.servo.getPosition());
                    backRight.servo.setPosition(frontRight.servo.getPosition());
                } else {
                    frontLeft.servo.setPosition(backLeft.servo.getPosition());
                    frontRight.servo.setPosition(backRight.servo.getPosition());
                }
            }
            //determine if target distance is reached
            int maxTraveled = Integer.MIN_VALUE;
            for (int i = 0; i < 4; i++) {
                maxTraveled = Math.abs(Math.max(maxTraveled, wheels[i].motor.getCurrentPosition() - startingCount[i]));
            }
            if (distance - maxTraveled < 10)
                break;
        }
        for (WheelAssembly wheel : wheels) wheel.motor.setPower(0);
        driveMode = DriveMode.STOP;
    }

    /**
     * Drive using currently specified power and heading values
     *
     * @param power     -1 to 1
     * @param heading   -90 to 90; relative to current robot orientation
     * @param allWheels <code>true</code> to use all 4 wheels,
     *                  <code>false</code> to use front wheels only
     */
    public void driveAndSteer(double power, double heading, boolean allWheels) throws InterruptedException {
        debug("driveSteer(pwr: %.3f, head: %.1f)", power, heading);
        if (power < -1 || power > 1) {
            throw new IllegalArgumentException("Power must be between -1 and 1");
        }
        if (heading < -90 || heading > 90) {
            throw new IllegalArgumentException("Heading must be between -90 and 90");
        }
        if (driveMode != DriveMode.STEER) {
            if (driveMode != DriveMode.STOP) reset();
            driveMode = DriveMode.STEER;
            for (WheelAssembly wheel : wheels)
                wheel.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        double[] newServoPositions = new double[4];
        if (allWheels) {
            Arrays.fill(newServoPositions, heading);
        } else if (power > 0) { // driving forward
            // front left and right
            newServoPositions[0] = newServoPositions[1] = heading / 2;
        } else if (power < 0) { // driving backward
            // back left and right
            newServoPositions[2] = newServoPositions[3] = heading / 2;
        }
        changeServoPositions(newServoPositions);

        for (WheelAssembly wheel : wheels) {
            wheel.motor.setPower(scalePower(power));
        }
    }

    public void driveAndSteerAuto(double power, double distance, double angle) throws InterruptedException {
        int[] startingCount = new int[4];
        for (int i = 0; i < 4; i++) {
            startingCount[i] = wheels[i].motor.getCurrentPosition();
            wheels[i].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        //tl.addLine(String.format("DriveAndSteer/pwr: %.3f, currentHeading: %.1f)", power, orientationSensor.getHeading()));
        //tl.update();
        long startTime = System.currentTimeMillis();
        driveAndSteer(power, angle, true);
        while (true) {
//            debug("DriveAndSteer/pwr: %.3f, currentHeading: %.1f)", power, orientationSensor.getHeading());
            if (System.currentTimeMillis() - startTime > 7000)
                throw new RuntimeException("Time Out");
            //tl.addLine(String.format("DriveAndSteer/pwr: %.3f, currentHeading: %.1f)", power, orientationSensor.getHeading()));
            //tl.update();
            int maxTraveled = Integer.MIN_VALUE;
            for (int i = 0; i < 4; i++) {
                maxTraveled = Math.max(maxTraveled, wheels[i].motor.getCurrentPosition() - startingCount[i]);
            }
            if (distance - maxTraveled < 10)
                break;
        }
        driveAndSteer(0, angle, true);
    }

    /**
     * Rotate in place using currently specified power
     */
    public void rotate(double power) throws InterruptedException {
        debug("rotate(pwr: %.3f)", power);
        if (power < -1 || power > 1) {
            throw new IllegalArgumentException("Power must be between -1 and 1");
        }
        if (driveMode != DriveMode.ROTATE) {
            if (driveMode != DriveMode.STOP) {
                // reset motors if they're moving; servos are adjusted below
                for (WheelAssembly wheel : wheels) wheel.reset(false);
            }
            driveMode = DriveMode.ROTATE;

            for (WheelAssembly wheel : wheels) {
                wheel.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // angle between Y axis and line from the center of chassis,
            //  which is assumed to be at (0, 0) to the center of the front right wheel
            double angle = Math.atan2(track, wheelBase) / Math.PI * 180;
            double[] newServoPositions = new double[4];
            // front left and back right
            newServoPositions[0] = newServoPositions[3] = angle;
            // front right and back left
            newServoPositions[1] = newServoPositions[2] = -1 * angle;
            changeServoPositions(newServoPositions);
        }

        frontLeft.motor.setPower(scalePower(power));
        frontRight.motor.setPower(-1 * scalePower(power));
        backLeft.motor.setPower(scalePower(power));
        backRight.motor.setPower(-1 * scalePower(power));
    }

    public void rotateDegree(double power, double deltaD) throws InterruptedException {
        double iniHeading = orientationSensor.getHeading();
        double finalHeading = iniHeading + deltaD;
        for (WheelAssembly wheel : wheels)
            wheel.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate(Math.signum(deltaD) * power);
        while (true) {
            if (Math.abs(finalHeading - orientationSensor.getHeading()) < 1.0)
                break;
        }
        for (WheelAssembly wheel : wheels)
            wheel.motor.setPower(0);
    }

    public void rotateTo(double power, double finalHeading, Telemetry tl) throws InterruptedException {
        double iniHeading = orientationSensor.getHeading();
        double deltaD = finalHeading - iniHeading;
        for (WheelAssembly wheel : wheels)
            wheel.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tl.addLine(String.format("rotateTo/pwr: %.3f, currentHeading: %.1f)", power, orientationSensor.getHeading()));
        tl.update();
        rotate(Math.signum(deltaD) * power);
        while (true) {
            tl.addLine(String.format("rotateTo/pwr: %.3f, currentHeading: %.1f)", power, orientationSensor.getHeading()));
            tl.update();
            if (Math.abs(finalHeading - orientationSensor.getHeading()) < 0.5)
                break;
        }
        for (WheelAssembly wheel : wheels)
            wheel.motor.setPower(0);
    }

    /**
     * Scales power according to <code>minPower</code> and <code>maxPower</code> settings
     */
    private double scalePower(double power) {
        return Math.signum(power) * minPower + power * (maxPower - minPower);
    }

    /**
     * Adjusts servo positions and waits for them to turn
     *
     * @param newPositions new servo positions matching wheel assembly order:
     *                     front left, front right, back left, back right
     */
    private void changeServoPositions(double[] newPositions) throws InterruptedException {
        double maxServoAdjustment = 0;
        for (int index = 0; index < newPositions.length; index++) {
            double servoAdjustment = Math.abs(newPositions[index] - wheels[index].servo.getPosition());
            maxServoAdjustment = Math.max(maxServoAdjustment, servoAdjustment);
            wheels[index].servo.setPosition(newPositions[index]);
        }
        Thread.sleep((int) Math.round(2 * maxServoAdjustment));
    }

    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        line.addData("Pwr", "%.2f", new Func<Double>() {
            @Override
            public Double value() {
                return frontLeft.motor.getPower();
            }
        });
        orientationSensor.setupTelemetry(line);

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
        line = telemetry.addLine("Srv: ");
        for (WheelAssembly wheel : wheels) {
            final AdjustableServo servo = wheel.servo;
            line.addData(wheel.position, "%+.1f", new Func<Double>() {
                @Override
                public Double value() {
                    return servo.getPosition();
                }
            });
        }
    }

    final class WheelAssembly {
        AdjustableServo servo;
        DcMotor motor;
        String position;

        WheelAssembly(Configuration configuration, String position, DcMotor.Direction direction) {
            // abbreviate position, leaving only capital letters
            StringBuilder sb = new StringBuilder(position);
            int index = 0;
            while (index < sb.length()) {
                if (Character.isUpperCase(sb.charAt(index))) {
                    index++;
                } else {
                    sb.deleteCharAt(index);
                }
            }
            this.position = sb.toString();

            servo = new AdjustableServo().configureLogging(
                    logTag + ":servo" + this.position, logLevel
            );
            servo.configure(configuration.getHardwareMap(), "servo" + position);
            configuration.register(servo);

            // motor = configuration.getHardwareMap().get(DcMotor.class, "motor" + position);
            motor = configuration.getHardwareMap().tryGet(DcMotor.class, "motor" + position);
            if (motor != null) motor.setDirection(direction);
        }

        void reset(boolean resetServo) {
            motor.setPower(0.0d);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (resetServo) servo.reset();
        }
    }
}
