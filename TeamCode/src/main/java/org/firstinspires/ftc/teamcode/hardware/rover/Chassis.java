package org.firstinspires.ftc.teamcode.hardware.rover;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.SwerveSystem;
import org.firstinspires.ftc.teamcode.hardware.rover.Core;

// TODO: Remove all of the NB_ prefixes from the variables


// Keep ordering consistent for defining, initializing, mapping, etc
public class Chassis {

    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;

    public Servo servoFrontLeft = null;
    public Servo servoFrontRight = null;
    public Servo servoBackLeft = null;
    public Servo servoBackRight = null;

    public DistanceSensor rangeSensor;

    public BNO055IMU imu = null;
    public BNO055IMU imu2 = null;
    public Acceleration accel = null;
    public Orientation angles;

    boolean use_imu2 = false;
    
    // Values for swerve mode
    public enum CarMode {
        CAR,
        CRAB,
        TURN,
        ORBIT
    }
    // Current mode & previous mode
    public CarMode cur_mode = CarMode.CAR;
    public CarMode old_mode = CarMode.CAR;

    /* variables for swerve positions */
    final static double NB_WIDTH_BETWEEN_WHEELS = 11.75;
    final static double NB_LENGTH_BETWEEN_WHEELS = 10.75;
    final static double NB_DISTANCE_FOR_ORBIT = 6.5;
    final static double MIN_TURNING_RADIUS = 13;
    final static double MAX_TURNING_RADIUS = 100;
    final static double NB_THETA_FRONT = (Math.atan(NB_DISTANCE_FOR_ORBIT / (0.5 * NB_WIDTH_BETWEEN_WHEELS))) * (180 / Math.PI);
    final static double NB_THETA_BACK = (Math.atan((NB_DISTANCE_FOR_ORBIT + NB_LENGTH_BETWEEN_WHEELS) / (0.5 * NB_WIDTH_BETWEEN_WHEELS))) * (180 / Math.PI);
    static double NB_SPOT_TURN_ANGLE_OFFSET = (Math.atan((0.32 * NB_LENGTH_BETWEEN_WHEELS) / (0.32 * NB_WIDTH_BETWEEN_WHEELS))) * (1.0 / (Math.PI));

    static double NB_CRAB_DIFF_DEC_FL = 0.317;
    static double NB_CRAB_DIFF_INC_FR = 0.316;
    static double NB_CRAB_DIFF_INC_BL = 0.316;
    static double NB_CRAB_DIFF_DEC_BR = 0.317;
    static double NB_LEFT_SV_DIFF = 0.000;
    static double NB_RIGHT_SV_DIFF = 0.000;

    final static double NB_SERVO_FL_FORWARD_POSITION = 0.5189;
    final static double NB_SERVO_FR_FORWARD_POSITION = 0.5106;
    final static double NB_SERVO_BL_FORWARD_POSITION = 0.4833;
    final static double NB_SERVO_BR_FORWARD_POSITION = 0.4861;

    static double SERVO_FL_STRAFE_POSITION = NB_SERVO_FL_FORWARD_POSITION - NB_CRAB_DIFF_DEC_FL - NB_LEFT_SV_DIFF;
    static double SERVO_FR_STRAFE_POSITION = NB_SERVO_FR_FORWARD_POSITION + NB_CRAB_DIFF_INC_FR + NB_RIGHT_SV_DIFF;
    static double SERVO_BL_STRAFE_POSITION = NB_SERVO_BL_FORWARD_POSITION + NB_CRAB_DIFF_INC_BL - NB_LEFT_SV_DIFF;
    static double SERVO_BR_STRAFE_POSITION = NB_SERVO_BR_FORWARD_POSITION - NB_CRAB_DIFF_DEC_BR + NB_RIGHT_SV_DIFF;

    static double SERVO_FL_TURN_POSITION = NB_SERVO_FL_FORWARD_POSITION - (NB_SPOT_TURN_ANGLE_OFFSET);
    static double SERVO_FR_TURN_POSITION = NB_SERVO_FR_FORWARD_POSITION + (NB_SPOT_TURN_ANGLE_OFFSET);
    static double SERVO_BL_TURN_POSITION = NB_SERVO_BL_FORWARD_POSITION + (NB_SPOT_TURN_ANGLE_OFFSET);
    static double SERVO_BR_TURN_POSITION = NB_SERVO_BR_FORWARD_POSITION - (NB_SPOT_TURN_ANGLE_OFFSET);

    static double SERVO_FL_ORBIT_POSITION = NB_SERVO_FL_FORWARD_POSITION - (NB_THETA_FRONT / 180.0);
    static double SERVO_FR_ORBIT_POSITION = NB_SERVO_FR_FORWARD_POSITION + (NB_THETA_FRONT / 180.0);
    static double SERVO_BL_ORBIT_POSITION = NB_SERVO_BL_FORWARD_POSITION - (NB_THETA_BACK / 180.0);
    static double SERVO_BR_ORBIT_POSITION = NB_SERVO_BR_FORWARD_POSITION + (NB_THETA_BACK / 180.0);
    
    public double target_heading = 0.0;

    final Core core;

    Chassis(Core c) {
        core = c;
    }

    public void init(HardwareMap hwMap) {
        motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hwMap.dcMotor.get("motorBackLeft");
        motorBackRight = hwMap.dcMotor.get("motorBackRight");

        servoFrontRight = hwMap.servo.get("servoFrontRight");
        servoFrontLeft = hwMap.servo.get("servoFrontLeft");
        servoBackLeft = hwMap.servo.get("servoBackLeft");
        servoBackRight = hwMap.servo.get("servoBackRight");

        rangeSensor = hwMap.get(DistanceSensor.class, "rangeSensor");

        // Initialize IMU
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
        accel = imu.getAcceleration();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Use IMU2 if IMU fails
        if (imu.getSystemStatus()== BNO055IMU.SystemStatus.SYSTEM_ERROR && imu2!=null)
            use_imu2 = true;

        // Invert all of these if using AndyMark motors
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power and set all servos to central position
        set_chassis_forward_position();
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        // Enable encoders
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Enable braking function
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double imu_heading() {
        if (!use_imu2)
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        else
            angles = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }

    void set_chassis_forward_position() {
            servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION);
            servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION);
            servoBackLeft.setPosition(NB_SERVO_BL_FORWARD_POSITION);
            servoBackRight.setPosition(NB_SERVO_BR_FORWARD_POSITION);
    }


    // Rename drive_distance
    public void driveTT(double lp, double rp) {
            double cur_heading = imu_heading();
            double heading_off_by = ((cur_heading - target_heading) / 360);
            if (cur_mode == CarMode.CAR) {
                if (rp > 0 && lp > 0) { // Forwards
                    if (cur_heading - target_heading > 0.7) {
                        servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION - heading_off_by);
                        servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION - heading_off_by);
                    } else if (cur_heading - target_heading < -0.7) {
                        servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION + heading_off_by);
                        servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION + heading_off_by);
                    }
                } else { // Backwards
                    if (cur_heading - target_heading > 0.7) { //Drifting to the left
                        servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION + heading_off_by); //Turn Front servos to the right
                        servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION + heading_off_by);
                    } else if (cur_heading - target_heading < -0.7) { //Drifting to the right
                        servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION - heading_off_by); //Turn Front servos to the left
                        servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION - heading_off_by);
                    }
                }
            } else if (cur_mode == CarMode.CRAB) {  //Tentative, could stand to remove
                if (cur_heading - target_heading > 0.7) { // crook to left,  slow down right motor
                    if (rp > 0) rp *= 0.7; //If the robot is going forward
                    else lp *= 0.7; // If the robot is going backwards
                } else if (cur_heading - target_heading < -0.7) { // crook to right, slow down left motor
                    if (lp > 0) lp *= 0.7;
                    else rp *= 0.7;

            }
        }
        if (cur_mode == CarMode.CAR) {
            motorFrontRight.setPower(rp);
            motorFrontLeft.setPower(lp);
            motorBackLeft.setPower(lp);
            motorBackRight.setPower(rp);
        } else if (cur_mode == CarMode.CRAB) {
            motorFrontLeft.setPower(lp);
            motorFrontRight.setPower(-lp);
            motorBackLeft.setPower(-rp);
            motorBackRight.setPower(rp);
        } else if (cur_mode == CarMode.TURN) {
            motorFrontRight.setPower(rp);
            motorFrontLeft.setPower(lp);
            motorBackLeft.setPower(lp);
            motorBackRight.setPower(rp);
        }
    }

    public void show_telemetry(Telemetry telemetry) {
        telemetry.addData("range", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("IMU", imu_heading());
    }
}