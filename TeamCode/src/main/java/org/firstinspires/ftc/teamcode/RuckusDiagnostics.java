package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.support.diagnostics.DiagnosticsTeleOp;
import org.firstinspires.ftc.teamcode.hardware.ruckus.ToboRuckus;

/**
 * Diagnostic TeleOp for Ruckus
 * @see DiagnosticsTeleOp
 */
@TeleOp(name="Ruckus::Diagnostics", group="Ruckus")
public class RuckusDiagnostics extends DiagnosticsTeleOp {

    // override log level, if desired
    // static { LOG_LEVEL = Log.VERBOSE; }

    @Override
    public Robot createRobot() {
        return new ToboRuckus().configureLogging("ToboRuckus", LOG_LEVEL);
    }
}
