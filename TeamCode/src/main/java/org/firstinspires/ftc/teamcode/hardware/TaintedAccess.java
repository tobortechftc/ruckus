package org.firstinspires.ftc.teamcode.hardware;

/**
 * Holding class for methods that access objects in ways that should be disallowed.
 */
public class TaintedAccess {

    private GreenMamba greenMamba;

    TaintedAccess(GreenMamba greenMamba) {
        this.greenMamba = greenMamba;
    }

//    SwerveUtilLOP getSwerveUtilLOP() {
//        if (this.greenMamba.swerveUtilLOP == null) {
//            throw new IllegalStateException ("Internal TaintedAccess.swerveUtilLOP object must be set. ");
//        }
//        return this.greenMamba.swerveUtilLOP;
//    }

    void stop_chassis() {
        greenMamba.swerve.stop_chassis();
    }

    void intakeGateInit() {
        greenMamba.intake.intakeGateInit();
    }

}
