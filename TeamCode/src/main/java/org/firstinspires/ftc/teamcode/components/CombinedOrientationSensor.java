package org.firstinspires.ftc.teamcode.components;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.support.Logger;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Orientation sensor implementation that combines / averages readings from multiple IMUs
 */
public class CombinedOrientationSensor extends Logger<CombinedOrientationSensor> {
    private static final double MAX_READING_DEVIATION = 10; // 10 degrees

    private Map<String, BNO055IMU> sensors;
    private boolean correctionEnabled = false;
    private Double lastReading = null;

    /**
     * Configures the hardware
     */
    public void configure(HardwareMap hardwareMap, String... deviceNames) {
        sensors = new LinkedHashMap<>();
        for (String deviceName : deviceNames) {
            BNO055IMU sensor = hardwareMap.tryGet(BNO055IMU.class, deviceName);
            if (sensor == null) {
                warn("%s is not available", deviceName);
                continue;
            }
            sensors.put(deviceName, sensor);
        }
        if (sensors.isEmpty()) {
            warn("No orientation sensors were found!");
            return;
        }

        reset();
    }

    public void reset() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        // FIXME: do we have a calibration data file? And is acceleration integration used?
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = this.logLevel < Log.WARN;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        for (Map.Entry<String, BNO055IMU> entry : sensors.entrySet()) {
            parameters.loggingTag = entry.getKey();
            entry.getValue().initialize(parameters);
        }
    }


    /**
     * Enables or disables correction mode. If enabled, sensor readings will be compared
     * to previous result and ignored if they deviate from it for more than <code>MAX_READING_DEVIATION</code>.
     * If disabled, current average reading will be returned.
     * Should only be enabled when robot is moving in a straight line.
     *
     * @param enable <code>true</code> to enable correction mode
     */
    public void enableCorrections(boolean enable) {
        if (this.correctionEnabled == enable) return;
        this.correctionEnabled = enable;
        this.lastReading = null;
    }

    /**
     * Returns current heading, averaged from reading of all available sensors.
     *
     * @return heading in the range of -180 to 180 degrees relative to robot
     * orientation during sensor initialization, with angles being measured
     * left (for negative values) and right (for positive values) of Y axis
     * with center of coordinates being the center of robot chassis
     * @see CombinedOrientationSensor#enableCorrections(boolean)
     */
    public double getHeading() {
        if (sensors.isEmpty()) return 0.0d;

        double reading1 = hardwareReading(sensors.get("imu"));
        double reading2 = hardwareReading(sensors.get("imu2"));
        return reading1 * reading2 < -100 ? reading1 : (reading1 + reading2) / 2;

//        return hardwareReading(sensors.get("imu2"));

        /*
        // get readings from all sensors and calculate an average
        double[] readings = new double[sensors.size()];
        double avgReading = 0.0d;
        int index = 0; boolean positive=true;
        for (Map.Entry<String, BNO055IMU> entry: sensors.entrySet()) {
            double value = hardwareReading(entry.getValue());
            if (index==0) {
                value *= -1.0; // first imu is reverted
                if (value<0) positive=false;
            }
            verbose("%s: %+.3f", entry.getKey(), value);
            readings[index++] = Math.abs(value);
            avgReading += value;
        }
        avgReading /= readings.length;
        if (!positive)
            avgReading *= -1.0;
        verbose("avg: %+.3f", avgReading);
        if (!correctionEnabled) {
            lastReading = avgReading;
            return avgReading;
        }

        if (lastReading!=null) {
            // drop all readings whose deviation from last reading is above threshold
            //  and recalculate average reading
            avgReading = 0.0d;
            int numReadings = 0;
            for (double reading : readings) {
                if (Math.abs(reading - lastReading) > MAX_READING_DEVIATION) continue;
                avgReading += reading;
                numReadings++;
            }
            verbose("last: %+.3f, num: %d, sum: %+.3f", lastReading, numReadings, avgReading);
            // return last reading if all current readings are above threshold
            if (numReadings > 0) lastReading = avgReading / numReadings;
            return lastReading;
        }

        // last reading is not available, set it to current average
        lastReading = avgReading;
        return lastReading;
        */
    }

    public void setupTelemetry(Telemetry.Line line) {
        if (sensors.isEmpty()) {
            line.addData("imu", "Not Found").setRetained(true);
        } else {
            for (Map.Entry<String, BNO055IMU> entry : sensors.entrySet()) {
                final BNO055IMU imu = entry.getValue();
                line.addData(entry.getKey(), "%.2f", new Func<Double>() {
                    @Override
                    public Double value() {
                        return hardwareReading(imu);
                    }
                });
            }
            line.addData("Heading", new Func<String>() {
                @Override
                public String value() {
                    return String.format("%.2f", getHeading());
                }
            });
        }
    }

    private double hardwareReading(BNO055IMU sensor) {
        Orientation orientation = sensor.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        );
        // hardware sensor returns positive angle for left deviation
        //  and negative for right deviation from Y axis, so it's reversed here
        return -1 * orientation.firstAngle;
    }
}
