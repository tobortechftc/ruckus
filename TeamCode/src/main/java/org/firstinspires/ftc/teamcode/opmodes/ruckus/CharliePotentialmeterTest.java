package org.firstinspires.ftc.teamcode.opmodes.ruckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.math.BigDecimal;

/**
 * Created by 28761 on 12/22/2018.
 */

@Autonomous(name = "Titan :: potential meter", group = "Titan")
public class CharliePotentialmeterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput potentiometer = hardwareMap.analogInput.get("pm1");
        coefficients = new BigDecimal[6];
        coefficients[0] = new BigDecimal("0.113543");
        coefficients[1] = new BigDecimal("1.4075");
        coefficients[2] = new BigDecimal("8.87737E-3");
        coefficients[3] = new BigDecimal("-1.17293E-4");
        coefficients[4] = new BigDecimal("4.5348E-7");
        coefficients[5] = new BigDecimal("-5.9825E-10");

        waitForStart();
        while (opModeIsActive()) {
            double value = getDegree(potentiometer);
            telemetry.addLine(String.format("degree: %f", value));
            telemetry.update();
            Thread.sleep(50);
        }

    }

    BigDecimal[] coefficients;

    public double getDegree(AnalogInput potentiometer) {
        BigDecimal reading = new BigDecimal(270 * (potentiometer.getVoltage() - 0.001) / (3.35 - 0.001));
        BigDecimal fx = BigDecimal.ZERO;
        for (int i = 5; i >= 0; i--) {
            fx = fx.multiply(reading);
            fx = fx.add(coefficients[i]);
        }
        return fx.doubleValue();
    }
}
