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
    private double maxRange = 127; // max range sensor detectable

    private WheelAssembly frontLeft;
    private WheelAssembly frontRight;
    private WheelAssembly backLeft;
    private WheelAssembly backRight;
    // array contains the same wheel assemblies as above variables
    //  and is convenient to use when actions have to be performed on all 4
    private WheelAssembly[] wheels = new WheelAssembly[4];
    private CombinedOrientationSensor orientationSensor;

    private DistanceSensor frontRangeSensor;
    private DistanceSensor backRangeSensor;
    private DistanceSensor leftRangeSensor;
    private DistanceSensor rightRangeSensor;

    private DriveMode driveMode = DriveMode.STOP;      // current drive mode
    private double targetHeading;     // intended heading for DriveMode.STRAIGHT as reported by orientation sensor
    private double headingDeviation;  // current heading deviation for DriveMode.STRAIGHT as reported by orientation sensor
    private double servoCorrection;   // latest correction applied to leading wheels' servos to correct heading deviation

    private boolean useScalePower = true;//
    private boolean setImuTelemetry = false;//unless debugging, don't set telemetry for imu
    private boolean setRangeSensorTelemetry = false;//unless debugging, don't set telemetry for range sensor

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

    public void configure(Configuration configuration, boolean auto) {
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

        if (auto) {
            orientationSensor = new CombinedOrientationSensor().configureLogging(logTag + "-sensor", logLevel);
            orientationSensor.configure(configuration.getHardwareMap(), "imu", "imu2");


            frontRangeSensor = configuration.getHardwareMap().get(DistanceSensor.class, "front_range");
            backRangeSensor = configuration.getHardwareMap().get(DistanceSensor.class, "back_range");
            leftRangeSensor = configuration.getHardwareMap().get(DistanceSensor.class, "left_range");
            rightRangeSensor = configuration.getHardwareMap().get(DistanceSensor.class, "right_range");
        }

        // register chassis as configurable component
        configuration.register(this);
    }

    public double distanceToFront() {
        if (frontRangeSensor==null)
            return 0;
        double dist = frontRangeSensor.getDistance(DistanceUnit.CM);
        int count = 0;
        while (dist > maxRange && (++count) < 5) {
            try {
                Thread.sleep(40);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            dist = frontRangeSensor.getDistance(DistanceUnit.CM);
        }
        if (dist > maxRange)
            dist = maxRange;
        return dist;
    }

    public double distanceToBack() {
        if (backRangeSensor==null)
            return 0;
        double dist = backRangeSensor.getDistance(DistanceUnit.CM);
        int count = 0;
        while (dist > maxRange && (++count) < 5) {
            try {
                Thread.sleep(40);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            dist = backRangeSensor.getDistance(DistanceUnit.CM);
        }
        if (dist > maxRange)
            dist = maxRange;
        return dist;
    }

    public double distanceToLeft() {
        if (leftRangeSensor==null)
            return 0;
        double dist = leftRangeSensor.getDistance(DistanceUnit.CM);
        int count = 0;
        while (dist > maxRange && (++count) < 5) {
            try {
                Thread.sleep(40);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            dist = leftRangeSensor.getDistance(DistanceUnit.CM);
        }
        if (dist > maxRange)
            dist = maxRange;
        return dist;
    }

    public double distanceToRight(){
        if (rightRangeSensor==null)
            return 0;
        double dist = rightRangeSensor.getDistance(DistanceUnit.CM);
        int count = 0;
        while (dist > maxRange && (++count) < 5) {
            try {
                Thread.sleep(40);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            dist = rightRangeSensor.getDistance(DistanceUnit.CM);
        }
        if (dist > maxRange)
            dist = maxRange;
        return dist;
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
            double sensorHeading = (orientationSensor == null ? 0 : orientationSensor.getHeading());
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

    double TICKS_PER_CM = 16.86;

    //using the indicated absolute power to drive a certain distance at a certain heading
    public void driveStraightAuto(double power, double cm, double heading, int timeout) throws InterruptedException {
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

        if (distance < 0) {
            power = -power;
            distance = -distance;
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

        //start powering wheels
        for (WheelAssembly wheel : wheels) wheel.motor.setPower(power);

        //record time
        long iniTime = System.currentTimeMillis();

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
            //determine if time limit is reached
            if (System.currentTimeMillis() - iniTime > timeout)
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

        if (Math.abs(power) > 0) {
            // only adjust servo positions if power is applied
            double[] newServoPositions = new double[4];
            if (allWheels) {
            /*
                if (Math.abs(heading)==90) {
                    // check whether all servos are already at 90 (or -90) degrees
                    boolean samePosition = (frontLeft.servo.getPosition() == frontRight.servo.getPosition())
                            && (frontLeft.servo.getPosition() == backLeft.servo.getPosition())
                            && (frontLeft.servo.getPosition() == backRight.servo.getPosition())
                            && (Math.abs(frontLeft.servo.getPosition()) == 90);
                    // keep wheels pointed sideways and invert the power if needed
                    if (samePosition) {
                        power *= heading == frontLeft.servo.getPosition() ? 1 : -1;
                        heading = frontLeft.servo.getPosition();
                    }
                } */
                Arrays.fill(newServoPositions, heading);
            } else if (power > 0) { // driving forward
                // front left and right
                newServoPositions[0] = newServoPositions[1] = heading / 2;
                // back left and right
                newServoPositions[2] = newServoPositions[3] = -1 * heading / 2;
            } else if (power < 0) { // driving backward
                // back left and right
                newServoPositions[2] = newServoPositions[3] = heading / 2;
                // front left and right
                newServoPositions[0] = newServoPositions[1] = -1 * heading / 2;
            }
            changeServoPositions(newServoPositions);
        }
        for (WheelAssembly wheel : wheels) {
            wheel.motor.setPower(scalePower(power));
        }
    }

    @Deprecated
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

    public void driveAlongWall(double power, double distance, double wallDistance) {

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

        frontLeft.motor.setPower(useScalePower ? scalePower(power) : power);
        frontRight.motor.setPower(-1 * (useScalePower ? scalePower(power) : power));
        backLeft.motor.setPower(useScalePower ? scalePower(power) : power);
        backRight.motor.setPower(-1 * (useScalePower ? scalePower(power) : power));
    }

    @Deprecated
    public void rotateDegree(double power, double deltaD) throws InterruptedException {
        double iniHeading = orientationSensor.getHeading();
        double finalHeading = iniHeading + deltaD;
        if (finalHeading > 180)
            finalHeading -= 360;
        else if (finalHeading < -180)
            finalHeading += 360;
        rotateTo(power, finalHeading);
    }

    //final heading needs to be with in range(-180,180]
    public void rotateTo(double power, double finalHeading) throws InterruptedException {
        debug("rotateT0(pwr: %.3f, finalHeading: %.1f)", power, finalHeading);
        double iniHeading = orientationSensor.getHeading();
        double deltaD = finalHeading - iniHeading;
        debug("iniHeading: %.1f, deltaD: %.1f)", iniHeading, deltaD);
        //do not turn if the heading is close enough the target
        if (Math.abs(deltaD) < 0.5)
            return;
        //resolve the issue with +-180 mark
        if (Math.abs(deltaD) > 180) {
            finalHeading = finalHeading + (deltaD > 0 ? -360 : +360);
            deltaD = 360 - Math.abs(deltaD);
            deltaD = -deltaD;
            debug("Adjusted finalHeading: %.1f, deltaD: %.1f)", finalHeading, deltaD);
        }
        //break on reaching the target
        for (WheelAssembly wheel : wheels)
            wheel.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ensure the condition before calling rotate()
        driveMode = DriveMode.STOP;
        useScalePower = false;
        //***** routine to start the wheels ******//
        rotate(Math.signum(deltaD) * power);
        //***** End routine to start the wheels ******//
        //record heading for checking in while loop
        double lastReading = orientationSensor.getHeading();
        long iniTime = System.currentTimeMillis();
        while (true) {
            double currentHeading = orientationSensor.getHeading();
            //we cross the +-180 mark if and only if the product below is a very negative number
            if ((currentHeading * lastReading < -100.0) || (Math.abs(currentHeading - lastReading) > 180.0)) {
                //deltaD>0 => cross the mark clockwise; deltaD<0 => cross the mark anticlockwise
                finalHeading = finalHeading + (deltaD > 0 ? -360.0 : +360.0);
                debug("Crossing180, finalHeading: %.1f, deltaD:%.1f)", finalHeading, deltaD);
            }
            debug("currentHeading: %.1f, finalHeading: %.1f)", currentHeading, finalHeading);
            //if within acceptable range, terminate
            if (Math.abs(finalHeading - currentHeading) < 0.5)
                break;
            //if overshoot, terminate
            if (deltaD > 0 && currentHeading - finalHeading > 0)
                break;
            if (deltaD < 0 && currentHeading - finalHeading < 0)
                break;
            if (System.currentTimeMillis() - iniTime > 3000)
                break;
            lastReading = currentHeading;
        }
        for (WheelAssembly wheel : wheels)
            wheel.motor.setPower(0);
        driveMode = DriveMode.STOP;
        useScalePower = true;
    }

    /**
     * Scales power according to <code>minPower</code> and <code>maxPower</code> settings
     */
    private double scalePower(double power) {
        double adjustedPower = Math.signum(power) * minPower + power * (maxPower - minPower);
        return Math.abs(adjustedPower) > 1.0 ? Math.signum(adjustedPower) : adjustedPower;
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

        //set up imu telemetry
        if (orientationSensor != null && setImuTelemetry) {
            line.addData("imu", "%.1f", new Func<Double>() {
                @Override
                public Double value() {
                    return orientationSensor.getHeading();
                }
            });
            orientationSensor.setupTelemetry(line);
        }

        //set up range sensor telemetry
        if (setRangeSensorTelemetry) {
            line.addData("rangeR", "%.1f", new Func<Double>() {
                @Override
                public Double value() {
                    return distanceToRight();
                }
            });
            line.addData("rangeL", "%.1f", new Func<Double>() {
                @Override
                public Double value() {
                    return distanceToLeft();
                }
            });
            line.addData("rangeF", "%.1f", new Func<Double>() {
                @Override
                public Double value() {
                    return distanceToFront();
                }
            });
            line.addData("rangeB", "%.1f", new Func<Double>() {
                @Override
                public Double value() {
                    return distanceToBack();
                }
            });
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

    public void resetOrientation() {
        orientationSensor.reset();
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
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (resetServo) servo.reset();
        }
    }
}
