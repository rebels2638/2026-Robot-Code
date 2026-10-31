package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class IntakeTest {
    @Test
    void resolvePivotMovingUp_switchesDirectionWhenOutsideTolerance() {
        double toleranceRotations = 0.01;
        assertTrue(Intake.resolvePivotMovingUp(
            0.30,
            0.10,
            toleranceRotations,
            false
        ));
        assertFalse(Intake.resolvePivotMovingUp(
            0.10,
            0.30,
            toleranceRotations,
            true
        ));
    }

    @Test
    void resolvePivotMovingUp_preservesPreviousDirectionWithinToleranceHysteresisBand() {
        double toleranceRotations = 0.01;
        assertTrue(Intake.resolvePivotMovingUp(
            0.20,
            0.205,
            toleranceRotations,
            true
        ));
        assertFalse(Intake.resolvePivotMovingUp(
            0.20,
            0.195,
            toleranceRotations,
            false
        ));
    }
}

