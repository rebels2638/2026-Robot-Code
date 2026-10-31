package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;

class RobotStateTest {
    @Test
    void shouldAllowVisionGyroReset_trueWhenDisabled() {
        assertTrue(RobotState.shouldAllowVisionGyroReset(true, false));
    }

    @Test
    void shouldAllowVisionGyroReset_trueWhenHeldEnabled() {
        assertTrue(RobotState.shouldAllowVisionGyroReset(false, true));
    }

    @Test
    void shouldAllowVisionGyroReset_falseWhenEnabledAndNotHeld() {
        assertFalse(RobotState.shouldAllowVisionGyroReset(false, false));
    }

    @Test
    void shouldApplyVisionGyroReset_requiresAllowanceAndTimeout() {
        assertFalse(RobotState.shouldApplyVisionGyroReset(false, 5.0, 0.0, 1.0));
        assertFalse(RobotState.shouldApplyVisionGyroReset(true, 1.0, 0.0, 1.0));
        assertTrue(RobotState.shouldApplyVisionGyroReset(true, 1.01, 0.0, 1.0));
    }

    @Test
    void shouldSkipRedundantDisabledGyroReset_skipsWhenDisabledAndWithinTolerance() {
        assertTrue(RobotState.shouldSkipRedundantDisabledGyroReset(
            true,
            false,
            Rotation2d.fromDegrees(90.0),
            Rotation2d.fromDegrees(90.5)
        ));
    }

    @Test
    void shouldSkipRedundantDisabledGyroReset_doesNotSkipWhenHeadingErrorExceedsThreshold() {
        assertFalse(RobotState.shouldSkipRedundantDisabledGyroReset(
            true,
            false,
            Rotation2d.fromDegrees(90.0),
            Rotation2d.fromDegrees(92.0)
        ));
    }

    @Test
    void shouldSkipRedundantDisabledGyroReset_doesNotSkipWhenHeldByDriver() {
        assertFalse(RobotState.shouldSkipRedundantDisabledGyroReset(
            true,
            true,
            Rotation2d.fromDegrees(90.0),
            Rotation2d.fromDegrees(90.1)
        ));
    }
}
