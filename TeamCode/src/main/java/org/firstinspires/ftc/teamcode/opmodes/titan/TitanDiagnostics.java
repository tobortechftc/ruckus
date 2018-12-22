package org.firstinspires.ftc.teamcode.opmodes.titan;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.hardware.ruckus.ToboRuckus;
import org.firstinspires.ftc.teamcode.hardware.titan.ToboTitan;
import org.firstinspires.ftc.teamcode.support.diagnostics.DiagnosticsTeleOp;

/**
 * Diagnostic TeleOp for Ruckus
 * @see DiagnosticsTeleOp
 */
@TeleOp(name="Titan::Diagnostics", group="Titan")
public class TitanDiagnostics extends DiagnosticsTeleOp {

    // override log level, if desired
    // static { LOG_LEVEL = Log.VERBOSE; }

    @Override
    public Robot createRobot() {
        return new ToboTitan().configureLogging("ToboTitan", LOG_LEVEL);
    }
}
