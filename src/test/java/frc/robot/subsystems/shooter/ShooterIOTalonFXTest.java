package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import com.ctre.phoenix6.signals.InvertedValue;
import org.junit.jupiter.api.Test;

class ShooterIOTalonFXTest {
    private static final double EPS = 1e-9;

    @Test
    void calculateTurretPositionCommand_preservesSignedVelocity() {
        ShooterIOTalonFX.TurretPositionCommand command = ShooterIOTalonFX.calculateTurretPositionCommand(
            0.2,
            -1.0,
            0.0,
            1.0
        );

        assertEquals(0.2, command.positionRotations(), EPS);
        assertEquals(-1.0, command.velocityRotPerSec(), EPS);
    }

    @Test
    void calculateTurretPositionCommand_clampsRequestedPositionInsideSoftLimits() {
        ShooterIOTalonFX.TurretPositionCommand command = ShooterIOTalonFX.calculateTurretPositionCommand(
            1.4,
            0.5,
            0.0,
            1.0
        );

        assertEquals(1.0, command.positionRotations(), EPS);
        assertEquals(0.5, command.velocityRotPerSec(), EPS);
    }

    @Test
    void calculateTurretPositionCommand_treatsNonFiniteVelocityAsZero() {
        ShooterIOTalonFX.TurretPositionCommand command = ShooterIOTalonFX.calculateTurretPositionCommand(
            0.4,
            Double.NaN,
            0.0,
            1.0
        );

        assertEquals(0.4, command.positionRotations(), EPS);
        assertEquals(0.0, command.velocityRotPerSec(), EPS);
    }

    @Test
    void calculateFlywheelControlTarget_prefersLeaderWhenAvailable() {
        assertSame(
            ShooterIOTalonFX.FlywheelControlTarget.LEADER,
            ShooterIOTalonFX.calculateFlywheelControlTarget(true, true)
        );
        assertSame(
            ShooterIOTalonFX.FlywheelControlTarget.LEADER,
            ShooterIOTalonFX.calculateFlywheelControlTarget(true, false)
        );
    }

    @Test
    void calculateFlywheelControlTarget_fallsBackToFollowerWhenLeaderIsDisconnected() {
        assertSame(
            ShooterIOTalonFX.FlywheelControlTarget.FOLLOWER,
            ShooterIOTalonFX.calculateFlywheelControlTarget(false, true)
        );
    }

    @Test
    void calculateFlywheelControlTarget_returnsNoneWhenBothFlywheelMotorsAreDisconnected() {
        assertSame(
            ShooterIOTalonFX.FlywheelControlTarget.NONE,
            ShooterIOTalonFX.calculateFlywheelControlTarget(false, false)
        );
    }

    @Test
    void getFlywheelFollowerInvertedValue_opposesLeaderWhenConfigured() {
        assertSame(
            InvertedValue.CounterClockwise_Positive,
            ShooterIOTalonFX.getFlywheelFollowerInvertedValue(true, true)
        );
        assertSame(
            InvertedValue.Clockwise_Positive,
            ShooterIOTalonFX.getFlywheelFollowerInvertedValue(false, true)
        );
    }

    @Test
    void getFlywheelFollowerInvertedValue_matchesLeaderWhenConfiguredAligned() {
        assertSame(
            InvertedValue.Clockwise_Positive,
            ShooterIOTalonFX.getFlywheelFollowerInvertedValue(true, false)
        );
        assertSame(
            InvertedValue.CounterClockwise_Positive,
            ShooterIOTalonFX.getFlywheelFollowerInvertedValue(false, false)
        );
    }
}
