package org.firstinspires.ftc.teamcode.hardware.ruckus;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.components.SwerveChassis;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
import org.firstinspires.ftc.teamcode.support.diagnostics.MenuEntry;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

public class ToboRuckus extends Logger<ToboRuckus> implements Robot {
    private Telemetry telemetry;
    private SwerveChassis chassis;

    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    @Override
    public void configure(Configuration configuration, Telemetry telemetry) {
        this.telemetry = telemetry;

        chassis = new SwerveChassis().configureLogging("Swerve", logLevel);
        chassis.configure(configuration);
    }

    @Override
    public void reset() {
        chassis.reset();
    }

    @MenuEntry(label = "Drive Straight", group = "Chassis Test")
    public void testStraight(EventManager em) {
        telemetry.addLine().addData("(LS)", "Drive").setRetained(true)
                .addData("Hold [LB]/[RB]", "45 degree").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) throws InterruptedException {
                double power = Math.max(Math.abs(currentX), Math.abs(currentY));
                double heading = toDegrees(currentX, currentY);
                debug("testStraight(): x: %+.2f, y: %+.2f, pow: %+.3f, head: %+.1f",
                        currentX, currentY, power, heading);

                if (source.isPressed(Button.LEFT_BUMPER) || source.isPressed(Button.RIGHT_BUMPER)) {
                    // constrain to 45 degree diagonals
                    heading = Math.signum(heading) * (Math.abs(heading) < 90 ? 45 : 135);
                } else {
                    // constrain to 90 degrees
                    heading = Math.round(heading / 90) * 90;
                }

                // adjust heading / power for driving backwards
                if (heading > 90) {
                    chassis.driveStraight(-1.0 * power, heading - 180);
                } else if (heading < -90) {
                    chassis.driveStraight(-1.0 * power, heading + 180);
                } else {
                    chassis.driveStraight(power, heading);
                }
            }
        }, Events.Axis.BOTH, Events.Side.LEFT);
    }

    @MenuEntry(label = "Drive & Steer", group = "Chassis Test")
    public void testSteering(EventManager em) {
        telemetry.addLine().addData(" < (LS) >", "Steer").setRetained(true)
                .addData("(RS)", "Power").setRetained(true);
        telemetry.addLine().addData("[LT] / [RT]", "Rotate").setRetained(true)
                .addData("Hold [LB] / [RB]", "Crab").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);

        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                double heading = 90 * currentX;
                double power = source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY);
                debug("testSteering(): head: %+.1f, pwr: %+.2f", heading, power);
                chassis.driveAndSteer(power, heading);
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);

        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                double heading = 90 * source.getStick(Events.Side.LEFT, Events.Axis.X_ONLY);
                double power = currentY;
                if (source.isPressed(Button.LEFT_BUMPER)) {
                    debug("testSteering(): head: -90, pwr: %+.2f", heading, power);
                    chassis.driveStraight(power, -90);
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
                    debug("testSteering(): head: 90, pwr: %+.2f", heading, power);
                    chassis.driveStraight(power, 90);
                } else {
                    debug("testSteering(): head: %+.1f, pwr: %+.2f", heading, power);
                    chassis.driveAndSteer(power, heading);
                }
            }
        }, Events.Axis.Y_ONLY, Events.Side.RIGHT);

        em.onTrigger(new Events.Listener() {
            @Override
            public void triggerMoved(EventManager source, Events.Side side, float current, float change) throws InterruptedException {
                // do not rotate if the robot is currently moving
                if (source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY)!=0
                        || source.getStick(Events.Side.LEFT, Events.Axis.X_ONLY)!=0) return;
                double power = current * (side == Events.Side.LEFT ? -1 : 1);
                chassis.rotate(power);
            }
        }, Events.Side.LEFT, Events.Side.RIGHT);
    }

    @MenuEntry(label = "Rotate in Place", group = "Chassis Test")
    public void testRotate(EventManager em) {
        telemetry.addLine().addData(" < (LS) >", "Power").setRetained(true);
        chassis.setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 100);
        em.onStick(new Events.Listener() {
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX,
                                   float currentY, float changeY) throws InterruptedException {
                chassis.rotate(currentX);
            }
        }, Events.Axis.X_ONLY, Events.Side.LEFT);
    }

    /**
     * Returns angle (-180 to 180 degrees) between positive Y axis
     *  and a line drawn from center of coordinates to (x, y).
     * Negative values are to the left of Y axis, positive are to the right
     */
    private double toDegrees(double x, double y) {
        if (x == 0) return y>=0 ? 0 : 180;
        return Math.atan2(x, y) / Math.PI * 180;
    }
}
