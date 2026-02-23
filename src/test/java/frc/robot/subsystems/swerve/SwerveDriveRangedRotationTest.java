package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class SwerveDriveRangedRotationTest {
    @Test
    void resolveRangedRotationState_staysReturningUntilInsideNominalBuffer() {
        SwerveDrive.CurrentOmegaOverrideState state = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.RANGED_RETURNING,
            true,
            false
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_RETURNING, state);
    }

    @Test
    void resolveRangedRotationState_transitionsToNominalWhenBufferedRangeRecovered() {
        SwerveDrive.CurrentOmegaOverrideState state = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.RANGED_RETURNING,
            true,
            true
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_NOMINAL, state);
    }

    @Test
    void resolveRangedRotationState_entersReturningWhenOutsideRange() {
        SwerveDrive.CurrentOmegaOverrideState state = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.RANGED_NOMINAL,
            false,
            false
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_RETURNING, state);
    }

    @Test
    void resolveRangedRotationState_usesNominalWhenInRangeAndNotReturning() {
        SwerveDrive.CurrentOmegaOverrideState state = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.NONE,
            true,
            true
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_NOMINAL, state);
    }
}
