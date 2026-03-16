package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShooterTest {
    private static final double EPS = 1e-9;

    @Test
    void selectConnectedFlywheelVelocity_prefersLeaderWhenConnected() {
        assertEquals(
            91.0,
            Shooter.selectConnectedFlywheelVelocity(true, 91.0, true, 88.0),
            EPS
        );
    }

    @Test
    void selectConnectedFlywheelVelocity_fallsBackToFollowerWhenLeaderIsDisconnected() {
        assertEquals(
            88.0,
            Shooter.selectConnectedFlywheelVelocity(false, 91.0, true, 88.0),
            EPS
        );
    }

    @Test
    void selectConnectedFlywheelVelocity_returnsNaNWhenBothMotorsAreDisconnected() {
        assertTrue(Double.isNaN(Shooter.selectConnectedFlywheelVelocity(false, 0.0, false, 0.0)));
    }

    @Test
    void selectConnectedFlywheelVelocityOrDefault_usesDefaultWhenBothMotorsAreDisconnected() {
        double setpoint = 75.0;
        double tolerance = 8.0;

        double fallbackVelocity = Shooter.selectConnectedFlywheelVelocityOrDefault(
            false,
            0.0,
            false,
            0.0,
            setpoint + Math.abs(tolerance) + 0.01
        );
        assertTrue(Math.abs(fallbackVelocity - setpoint) > tolerance);
    }

    @Test
    void isFlywheelAtSetpoint_trueWhenFollowerAloneIsWithinTolerance() {
        assertTrue(Shooter.isFlywheelAtSetpoint(false, 0.0, true, 78.0, 75.0, 8.0));
    }

    @Test
    void isFlywheelAtSetpoint_falseWhenFollowerAloneIsOutsideTolerance() {
        assertFalse(Shooter.isFlywheelAtSetpoint(false, 0.0, true, 84.0, 75.0, 8.0));
    }

    @Test
    void isFlywheelAtSetpoint_falseWhenBothFlywheelMotorsAreDisconnected() {
        assertFalse(Shooter.isFlywheelAtSetpoint(false, 0.0, false, 0.0, 0.0, 8.0));
    }
}
