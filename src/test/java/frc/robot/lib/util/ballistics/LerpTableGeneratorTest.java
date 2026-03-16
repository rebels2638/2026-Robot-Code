package frc.robot.lib.util.ballistics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.configs.ShooterConfig;
import java.util.List;
import org.junit.jupiter.api.Test;

class LerpTableGeneratorTest {
    @Test
    // Generated table should include all default distances and stay inside configured hood limits.
    void generate_producesDefaultDistanceEntriesWithReasonableValues() {
        ShooterConfig config = new ShooterConfig();
        config.shooterPoseZ = 0.95;
        config.hoodMinAngleDegrees = 15.0;
        config.hoodMaxAngleDegrees = 80.0;
        config.flywheelRadiusMeters = 0.05;
        config.backRollerGearRatio = 0.65;
        config.backRollerRadiusMeters = 0.03;
        config.maxBallisticFlywheelVelocityRPS = 80.0;

        List<ShooterConfig.LerpEntry> entries = LerpTableGenerator.generate(config);

        assertEquals(13, entries.size());

        double previousDistance = -1.0;
        for (ShooterConfig.LerpEntry entry : entries) {
            assertTrue(entry.distanceMeters > previousDistance);
            assertTrue(entry.hoodAngleDegrees >= config.hoodMinAngleDegrees);
            assertTrue(entry.hoodAngleDegrees <= config.hoodMaxAngleDegrees);
            assertTrue(entry.flywheelVelocityRPS > 0.0);
            assertTrue(entry.flywheelVelocityRPS <= config.getMaxBallisticFlywheelVelocityRPS());
            assertTrue(entry.flightTimeSeconds > 0.0);
            previousDistance = entry.distanceMeters;
        }
    }

    @Test
    // Existing table distances should be preserved so per-config sampling can be regenerated.
    void generate_reusesConfiguredDistancesWhenPresent() {
        ShooterConfig config = new ShooterConfig();
        config.shooterPoseZ = 0.95;
        config.hoodMinAngleDegrees = 15.0;
        config.hoodMaxAngleDegrees = 80.0;
        config.flywheelRadiusMeters = 0.05;
        config.backRollerGearRatio = 0.65;
        config.backRollerRadiusMeters = 0.03;

        ShooterConfig.LerpEntry first = new ShooterConfig.LerpEntry();
        first.distanceMeters = 1.0;
        ShooterConfig.LerpEntry second = new ShooterConfig.LerpEntry();
        second.distanceMeters = 2.0;
        config.shootingLerpTable = List.of(first, second);

        List<ShooterConfig.LerpEntry> entries = LerpTableGenerator.generate(config);

        assertEquals(2, entries.size());
        assertEquals(1.0, entries.get(0).distanceMeters, 1e-9);
        assertEquals(2.0, entries.get(1).distanceMeters, 1e-9);
    }
}
