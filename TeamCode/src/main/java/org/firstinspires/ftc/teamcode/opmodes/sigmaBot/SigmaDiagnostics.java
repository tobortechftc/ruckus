package org.firstinspires.ftc.teamcode.opmodes.sigmaBot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.diagnostics.DiagnosticsTeleOp;

/**
 * Diagnostic TeleOp for Ruckus
 * @see DiagnosticsTeleOp
 */
@TeleOp(name="Sigma::Diagnostics", group="Sigma")
public class SigmaDiagnostics extends DiagnosticsTeleOp {

    // override log level, if desired
    // static { LOG_LEVEL = Log.VERBOSE; }

    @Override
    public Robot createRobot() {
        return new ToboSigma().configureLogging("ToboSigma", LOG_LEVEL);
    }
}
