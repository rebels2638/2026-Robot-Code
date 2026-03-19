package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

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
}
