package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;

/**
 * Wrapper around hardware Servo that allows left / center / right stops to be adjusted
 *  and translates -90 / 90 degree range to appropriate servo positions
 */
public class AdjustableServo extends Logger<AdjustableServo> implements Configurable {
    private String deviceName;
    private Servo servo;
    private double position = 0.0; // position in degrees

    private boolean adjustmentMode = false;
    private double left = 0.2;   // left servo stop
    private double center = 0.5; // central servo position
    private double right = 0.8;  // right servo stop

    public void configure(HardwareMap hardwareMap, String deviceName) {
        this.servo = hardwareMap.get(Servo.class, deviceName);
        this.servo.setDirection(Servo.Direction.REVERSE);
        this.deviceName = deviceName;
    }

    @Override
    public String getUniqueName() {
        return deviceName;
    }

    @Override
    public void setAdjustmentMode(boolean adjustmentMode) {
        this.adjustmentMode = adjustmentMode;
    }

    @Adjustable(min = 0.0f, max = 0.3f, step = 0.001f)
    public double getLeft() {
        return left;
    }
    public void setLeft(double left) {
        this.left = left;
        if (adjustmentMode) setPosition(-90);
    }

    @Adjustable(min = 0.4f, max = 0.6f, step = 0.001f)
    public double getCenter() {
        return center;
    }
    public void setCenter(double center) {
        this.center = center;
        if (adjustmentMode) setPosition(0);
    }

    @Adjustable(min = 0.7f, max = 1.0f, step = 0.001f)
    public double getRight() {
        return right;
    }
    public void setRight(double right) {
        this.right = right;
        if (adjustmentMode) setPosition(90);
    }

    /**
     * Returns servo position in degrees in -90 to +90 range with -90 being the leftmost position,
     *  0 being the center and +90 being the rightmost position.
     * Note that position returned is the last position set on servo and may not match its actual position.
     */
    public double getPosition() {
        return position;
    }

    /**
     * Sets servo position in degrees in -90 to +90 range with -90 being the leftmost position,
     *  0 being the center and +90 being the rightmost position.
     * @param degrees new position to turn servo to
     */
    public void setPosition(double degrees) {
        if (Math.abs(degrees) > 90) {
            throw new IllegalArgumentException("Servo position must be between -90 and 90 degrees");
        }
        double hwPosition; // actual hardware position set on servo
        if (degrees < 0) {
            // transform "-90 / 0" to "left / center" interval
            hwPosition = (degrees / 90.0 + 1.0) * (center - left) + left;
        } else {
            // transform "0 / 90" to "center / right" interval
            hwPosition = degrees / 90.0 * (right - center) + center;
        }
        verbose("position: %.2f -> %.2f, hw: %.2f", position, degrees, hwPosition);
        servo.setPosition(hwPosition);
        position = degrees;
    }

    /**
     * Adjusts current servo position by specified number of degrees. This is a convenience method
     *  that reads current servo position and adjusts it by specified amount making sure the result
     *  does not fall outside of -90 to +90 degree range.
     * @param degrees number of degrees (positive or negative) to adjust current servo position by
     */
    public void adjustPosition(double degrees) {
        double newPosition = getPosition() + degrees;
        newPosition = Math.signum(newPosition) * Math.min(Math.abs(newPosition), 90);
        setPosition(newPosition);
    }

    /**
     * Resets servo position back to center
     */
    public void reset() {
        position = 0;
        servo.setPosition(center);
    }
}
