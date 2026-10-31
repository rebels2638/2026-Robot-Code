package frc.robot.subsystems.swerve.module;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ModuleIOTalonFXTest {
    @Test
    void calculateDriveAccelerationMetersPerSec2_computesFiniteAccelerationForNominalDt() {
        assertEquals(
            5.0,
            ModuleIOTalonFX.calculateDriveAccelerationMetersPerSec2(2.1, 2.0, 0.02),
            1e-9
        );
    }

    @Test
    void calculateDriveAccelerationMetersPerSec2_returnsZeroOnZeroNegativeOrLargeDtOrNaN() {
        assertEquals(0.0, ModuleIOTalonFX.calculateDriveAccelerationMetersPerSec2(2.1, 2.0, 0.0), 1e-9);
        assertEquals(0.0, ModuleIOTalonFX.calculateDriveAccelerationMetersPerSec2(2.1, 2.0, -0.02), 1e-9);
        assertEquals(0.0, ModuleIOTalonFX.calculateDriveAccelerationMetersPerSec2(2.1, 2.0, 0.50), 1e-9);
        assertEquals(0.0, ModuleIOTalonFX.calculateDriveAccelerationMetersPerSec2(Double.NaN, 2.0, 0.02), 1e-9);
    }
}

