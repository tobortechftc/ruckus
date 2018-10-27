package org.firstinspires.ftc.teamcode.hardware.rover;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// TODO: Remove all of the NB_ prefixes from the variables


public class Chassis {

    ElapsedTime runtime = new ElapsedTime(); // variable runtime, can be reset
    ElapsedTime runtimeFull = new ElapsedTime(); // runtime that is started at beginning and not reset

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

    int leftCnt = 0; // left motor target counter for encoders [cart variable]
    int rightCnt = 0; // right motor target counter for encoders [cart variable]
    final static int ONE_ROTATION = 538; // for new AndyMark-20 motor encoder one rotation
    final static double RROBOT = 6.63;  // number of wheel turns to get chassis 360-degree setPosition
    final static double INCHES_PER_ROTATION = 12.69; // inches per chassis motor rotation based on 1:1 gear ratio



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
        setChassisForwardPosition();
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

    public double imuHeading() {
        if (!use_imu2)
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        else
            angles = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }

    void setChassisForwardPosition() {
            servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION);
            servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION);
            servoBackLeft.setPosition(NB_SERVO_BL_FORWARD_POSITION);
            servoBackRight.setPosition(NB_SERVO_BR_FORWARD_POSITION);
    }


    // Rename to drive_power
    public void driveTT(double lp, double rp) {
        double cur_heading = imuHeading();
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
        if (cur_mode == CarMode.CAR || cur_mode == CarMode.TURN) {
            motorFrontRight.setPower(rp);
            motorFrontLeft.setPower(lp);
            motorBackLeft.setPower(lp);
            motorBackRight.setPower(rp);
        } else if (cur_mode == CarMode.CRAB) {
            motorFrontLeft.setPower(lp);
            motorFrontRight.setPower(-lp);
            motorBackLeft.setPower(-rp);
            motorBackRight.setPower(rp);
        }
    }

    
    // Rename to drive_distance
    public void StraightCm(double power, double cm) throws InterruptedException {
        target_heading = imuHeading();
        double WHEEL_DIAMETER = 102.18; // In millimeters
        double WHEEL_RADIUS = WHEEL_DIAMETER / 2; // Still in millimeters
        double CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // Also is mm per rotation

        double numberR = cm / (CIRCUMFERENCE / 10);
        if (cur_mode == CarMode.CRAB) {
            StraightR(power, numberR);
        } else {
            StraightR(-power, numberR);
        }
    }

    public void driveTTCoast(double lp, double rp) {
        boolean strafeRight = false;
        if (lp > 0 && rp > 0) {
            strafeRight = true;
        } else {
            strafeRight = false;
        }
        if (cur_mode == CarMode.CAR) {
            motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else if (cur_mode == CarMode.CRAB && strafeRight) {
            motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else if (cur_mode == CarMode.CRAB && !strafeRight) {
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        double cur_heading = imuHeading();
        double heading_off_by = ((cur_heading - target_heading) / 360);
        if (cur_mode == CarMode.CAR) {
            if (rp > 0 && lp > 0) { //When going forward
                if (cur_heading - target_heading > 0.7) {
                    servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION - heading_off_by);
                    servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION - heading_off_by);
                } else if (cur_heading - target_heading < -0.7) {
                    servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION + heading_off_by);
                    servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION + heading_off_by);
                }
            } else { // When going backward
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
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }
        else if (cur_mode == CarMode.CRAB && strafeRight) {
            motorFrontRight.setPower(rp);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(lp);
        }
        else if (cur_mode == CarMode.CRAB) {
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(rp);
            motorBackLeft.setPower(lp);
            motorBackRight.setPower(0);
        }
        else if (cur_mode == CarMode.TURN) {
            motorFrontRight.setPower(rp);
            motorFrontLeft.setPower(lp);
            motorBackLeft.setPower(lp);
            motorBackRight.setPower(rp);
        }
    }

    void StraightR(double power, double n_rotations) throws InterruptedException {
        resetChassis();
        // set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = motorFrontLeft.getCurrentPosition();
        int rightEncode = motorFrontRight.getCurrentPosition();
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftCnt = (int) (ONE_ROTATION * n_rotations);
        rightCnt = (int) (ONE_ROTATION * n_rotations);
        setMotorPower((float) power);
        runUntilEncoder(leftCnt, power, rightCnt, power);
        if(cur_mode == CarMode.CRAB) {
            servoFrontLeft.setPosition(SERVO_FL_STRAFE_POSITION);
            servoFrontRight.setPosition(SERVO_FR_STRAFE_POSITION);
            servoBackLeft.setPosition(SERVO_BL_STRAFE_POSITION);
            servoBackRight.setPosition(SERVO_BR_STRAFE_POSITION);
        } else {
            setChassisForwardPosition();
        }

        //if (!fast_mode)
        //    sleep(135);
    }

    public void runUntilEncoder(int leftCnt, double leftPower, int rightCnt, double rightPower) throws InterruptedException {
        runtime.reset();
        int leftTC1 = leftCnt;
        int rightTC1 = rightCnt;
        int leftTC2 = 0;
        int rightTC2 = 0;
        int leftTC0 = 0;
        int rightTC0 = 0;
        int targetPosFrontLeft;
        int curPosFrontLeft = motorFrontLeft.getCurrentPosition();
        int targetPosFrontRight;
        int curPosFrontRight = motorFrontRight.getCurrentPosition();
        int targetPosBackLeft;
        int curPosBackLeft = (motorBackLeft!=null?motorBackLeft.getCurrentPosition():0);
        int targetPosBackRight;
        int curPosBackRight = (motorBackRight!=null?motorBackRight.getCurrentPosition():0);
        double initLeftPower = leftPower;
        double initRightPower = rightPower;
        double leftPowerSign = leftPower/Math.abs(leftPower);
        double rightPowerSign = rightPower/Math.abs(rightPower);
        boolean strafeRight = false;

        if(leftPower > 0 && rightPower > 0){
            strafeRight = true;
        }
        else{
            strafeRight = false;
        }

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
        if (Math.abs(leftPower) > 0.4 && leftTC1 > 600) {
            leftTC2 = 150;
            leftTC0 = 75;
            leftTC1 -= 225;
        }
        if (Math.abs(rightPower) > 0.4 && rightTC1 > 600) {
            rightTC2 = 150;
            rightTC0 = 75;
            rightTC1 -= 225;
        }
        if(cur_mode == CarMode.CAR) {
            if (rightTC0 > 0 || leftTC0 > 0) {
                targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC0);
                targetPosFrontRight = curPosFrontRight + ((int) rightPowerSign * rightTC0);
                motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                motorFrontRight.setTargetPosition(targetPosFrontRight);

                runtime.reset();
                driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && runtime.seconds() < 1) {
                    driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                    core.yield();
                    // showTelemetry();
                }
            }
            curPosFrontLeft = motorFrontLeft.getCurrentPosition();
            curPosFrontRight = motorFrontRight.getCurrentPosition();

            targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC1);
            targetPosFrontRight = curPosFrontRight + ((int) rightPowerSign * rightTC1);

            motorFrontLeft.setTargetPosition(targetPosFrontLeft);
            motorFrontRight.setTargetPosition(targetPosFrontRight);

            driveTTCoast(leftPower, rightPower);
            int iter = 0;
            int prev_lpos = curPosFrontLeft;
            double cur_time = runtime.nanoseconds()/1000000000.0;
            double prev_time = cur_time;
            double cur_speed = 0, prev_speed=0;
            while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && runtime.seconds() < 7) {
                driveTTCoast(leftPower, rightPower);
                core.yield();
                /*
                if (use_verbose) {
                    core.telemetry.addData("4.Speed cur/prev/i=", "%.2f/%.2f/%1d", cur_speed, prev_speed, iter);
                    core.telemetry.addData("5.time cur/prev/^=", "%.4f/%.4f/%.4f", cur_time, prev_time, (cur_time-prev_time));
                    core.telemetry.addData("6.enco cur/prev/^=", "%2d/%2d/%2d", curPosFrontLeft, prev_lpos,(curPosFrontLeft-prev_lpos));
                    core.telemetry.update();
                }
                */
            }

            if (rightTC2 > 0 || leftTC2 > 0) {
                curPosFrontLeft = motorFrontLeft.getCurrentPosition();
                curPosFrontRight = motorFrontRight.getCurrentPosition();

                targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC2);
                targetPosFrontRight = curPosFrontRight + ((int) rightPowerSign * rightTC2);

                motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                motorFrontRight.setTargetPosition(targetPosFrontRight);
                driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && runtime.seconds() < 8) {
                    driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                    core.yield();
                }
            }
            /*
            if (use_verbose) {
                //stop_chassis();
                core.telemetry.addData("4.Speed cur/prev/i/bumped=", "%.2f/%.2f/%1d/%s",
                        cur_speed, prev_speed, iter, (bump_detected?"T":"F"));
                core.telemetry.addData("5.time cur/prev/^=", "%.4f/%.4f/%.4f", cur_time, prev_time, (cur_time-prev_time));
                core.telemetry.addData("6.enco cur/prev/^=", "%2d/%2d/%2d", curPosFrontLeft, prev_lpos,(curPosFrontLeft-prev_lpos));
                core.telemetry.addLine("7.Hit B/X button to go next/exit ...");
                core.telemetry.update();
                //while (!gamepad1.x&&!gamepad1.b) {;}
            }
            */
        }
        else if (cur_mode == CarMode.CRAB) {
            if(strafeRight) {
                if (rightTC0 > 0 || leftTC0 > 0) {
                    targetPosFrontRight = curPosFrontRight + ((int) -leftPowerSign * leftTC0);
                    targetPosBackRight = curPosBackRight + ((int) -rightPowerSign * rightTC0);
                    motorFrontRight.setTargetPosition(targetPosFrontRight);
                    motorBackRight.setTargetPosition(targetPosBackRight);

                    runtime.reset();
                    driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                    while (motorFrontRight.isBusy() && motorBackRight.isBusy() && runtime.seconds() < 1) {
                        driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                        core.yield();
                        // showTelemetry();
                    }
                }
                curPosFrontRight = motorFrontRight.getCurrentPosition();
                curPosBackRight = motorBackRight.getCurrentPosition();

                targetPosFrontRight = curPosFrontRight + ((int) -leftPowerSign * leftTC1);
                targetPosBackRight = curPosBackRight + ((int) -rightPowerSign * rightTC1);

                motorFrontRight.setTargetPosition(targetPosFrontRight);
                motorBackRight.setTargetPosition(targetPosBackRight);

                driveTTCoast(leftPower, rightPower);
                while (motorFrontRight.isBusy() && motorBackRight.isBusy() && runtime.seconds() < 7) {
                    driveTTCoast(leftPower, rightPower);
                    core.yield();
                }

                if (rightTC2 > 0 || leftTC2 > 0) {
                    curPosFrontRight = motorFrontRight.getCurrentPosition();
                    curPosBackRight = motorBackRight.getCurrentPosition();

                    targetPosFrontRight = curPosFrontRight + ((int) -leftPowerSign * leftTC2);
                    targetPosBackRight = curPosBackRight + ((int) -rightPowerSign * rightTC2);

                    motorFrontRight.setTargetPosition(targetPosFrontRight);
                    motorBackRight.setTargetPosition(targetPosBackRight);
                    driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                    while (motorFrontRight.isBusy() && motorBackRight.isBusy() && runtime.seconds() < 8) {
                        driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                        core.yield();
                    }
                }
            }
            else{
                if (rightTC0 > 0 || leftTC0 > 0) {
                    targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC0);
                    targetPosBackLeft = curPosBackLeft + ((int) rightPowerSign * rightTC0);
                    motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                    motorBackLeft.setTargetPosition(targetPosBackLeft);

                    runtime.reset();
                    driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                    while (motorFrontLeft.isBusy() && motorBackLeft.isBusy() && runtime.seconds() < 1) {
                        driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                        core.yield();
                        // showTelemetry();
                    }
                }
                curPosFrontLeft = motorFrontLeft.getCurrentPosition();
                curPosBackLeft = motorBackLeft.getCurrentPosition();

                targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC1);
                targetPosBackLeft = curPosBackLeft + ((int) rightPowerSign * rightTC1);

                motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                motorBackLeft.setTargetPosition(targetPosBackLeft);

                driveTTCoast(leftPower, rightPower);
                while (motorFrontLeft.isBusy() && motorBackLeft.isBusy() && runtime.seconds() < 7) {
                    driveTTCoast(leftPower, rightPower);
                    core.yield();
                }

                if (rightTC2 > 0 || leftTC2 > 0) {
                    curPosFrontLeft = motorFrontLeft.getCurrentPosition();
                    curPosBackLeft = motorBackLeft.getCurrentPosition();

                    targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC2);
                    targetPosBackLeft = curPosBackLeft + ((int) rightPowerSign * rightTC2);

                    motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                    motorBackLeft.setTargetPosition(targetPosBackLeft);
                    driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                    while (motorFrontLeft.isBusy() && motorBackLeft.isBusy() && runtime.seconds() < 8) {
                        driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                        core.yield();
                    }
                }
            }
        }
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorPower(0);
        runtime.reset();
    }
    
    // Rename to reset_encoders
    void resetChassis()  {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftCnt = 0;
        rightCnt = 0;
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorPower(double power) {
        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
    }

    public void showTelemetry(Telemetry telemetry) {
        telemetry.addData("range", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("IMU", imuHeading());
    }
}