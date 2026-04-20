package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.gson.Gson;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.configs.ShooterConfig;
import frc.robot.lib.util.ballistics.BallisticsConstants;
import frc.robot.lib.util.ballistics.BallisticsPhysics;
import frc.robot.lib.util.ballistics.BallisticsModel;
import frc.robot.lib.util.ballistics.TrajectoryResult;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class ShotLerpDiagnosticsTest {
    @Test
    // Regression: runtime visualization kinematics should match the direct wheel model at the table setpoint.
    void simLerpTable_runtimeVisualizationKinematicsMatchWheelModel() throws IOException {
        ShooterConfig config = loadShooterConfig("src/main/deploy/configs/shooter/sim.json");
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable = config.getLerpTable();
        var shootingEntries = config.getShootingLerpEntries();

        for (ShooterConfig.LerpEntry entry : shootingEntries) {
            double exitVelocity = ShotCalculator.calculateExitVelocityMetersPerSec(
                entry.distanceMeters,
                lerpTable,
                entry.flywheelVelocityRPS,
                measuredRps -> calculateShotExitVelocityMetersPerSec(measuredRps, config)
            );
            double spinRateRadPerSec = ShotCalculator.calculateSpinRateRadPerSec(
                entry.distanceMeters,
                lerpTable,
                entry.flywheelVelocityRPS,
                measuredRps -> calculateSpinRadPerSec(measuredRps, config)
            );

            assertEquals(
                calculateShotExitVelocityMetersPerSec(entry.flywheelVelocityRPS, config),
                exitVelocity,
                1e-9
            );
            assertEquals(calculateSpinRadPerSec(entry.flywheelVelocityRPS, config), spinRateRadPerSec, 1e-9);
        }
    }

    @Test
    // Sim shooting table should match the simple gravity-only model configured for simulation.
    void simShootingLerpTable_matchesConfiguredBallisticsModel() throws IOException {
        ShooterConfig config = loadShooterConfig("src/main/deploy/configs/shooter/sim.json");
        assertEquals(BallisticsModel.SIMPLE, config.getBallisticsModel());
        assertTrue(config.getShootingLerpEntries() != null && !config.getShootingLerpEntries().isEmpty());

        for (ShooterConfig.LerpEntry entry : config.getShootingLerpEntries()) {
            double exitVelocity = calculateShotExitVelocityMetersPerSec(entry.flywheelVelocityRPS, config);
            double spinRateRadPerSec = calculateSpinRadPerSec(entry.flywheelVelocityRPS, config);

            TrajectoryResult result = BallisticsPhysics.simulateToDistance(
                entry.distanceMeters,
                Math.toRadians(entry.hoodAngleDegrees),
                exitVelocity,
                config.shooterPoseZ,
                1.8034,
                spinRateRadPerSec,
                0.002,
                config.getBallisticsModel()
            );

            assertTrue(result.reachedTarget());
            assertTrue(Math.abs(result.finalHeight() - 1.8034) < 0.01);
            assertTrue(Math.abs(result.flightTime() - entry.flightTimeSeconds) < 0.01);
            assertTrue(entry.hoodAngleDegrees >= config.hoodMinAngleDegrees);
            assertTrue(entry.hoodAngleDegrees <= config.hoodMaxAngleDegrees);
            assertTrue(entry.flywheelVelocityRPS <= config.getMaxBallisticFlywheelVelocityRPS());
        }
    }

    @Test
    // Sim pass table should also match the simple gravity-only model and stay within sim flywheel limits.
    void simPassLerpTable_matchesConfiguredBallisticsModel() throws IOException {
        ShooterConfig config = loadShooterConfig("src/main/deploy/configs/shooter/sim.json");
        assertEquals(BallisticsModel.SIMPLE, config.getBallisticsModel());
        assertTrue(config.passLerpTable != null && !config.passLerpTable.isEmpty());

        for (ShooterConfig.LerpEntry entry : config.passLerpTable) {
            double exitVelocity = calculateShotExitVelocityMetersPerSec(entry.flywheelVelocityRPS, config);
            double spinRateRadPerSec = calculateSpinRadPerSec(entry.flywheelVelocityRPS, config);

            TrajectoryResult result = BallisticsPhysics.simulateToDistance(
                entry.distanceMeters,
                Math.toRadians(entry.hoodAngleDegrees),
                exitVelocity,
                config.shooterPoseZ,
                0.0,
                spinRateRadPerSec,
                0.002,
                config.getBallisticsModel()
            );

            assertTrue(result.reachedTarget() || Math.abs(result.finalDistance() - entry.distanceMeters) < 0.02);
            assertTrue(Math.abs(result.finalHeight() - 0.0) < 0.01);
            assertTrue(Math.abs(result.flightTime() - entry.flightTimeSeconds) < 0.01);
            assertTrue(entry.hoodAngleDegrees >= config.hoodMinAngleDegrees);
            assertTrue(entry.hoodAngleDegrees <= config.hoodMaxAngleDegrees);
            assertTrue(entry.flywheelVelocityRPS <= config.getMaxBallisticFlywheelVelocityRPS());
        }
    }

    private static ShooterConfig loadShooterConfig(String path) throws IOException {
        return new Gson().fromJson(Files.readString(Path.of(path)), ShooterConfig.class);
    }

    private static double calculateShotExitVelocityMetersPerSec(double flywheelRps, ShooterConfig config) {
        double flywheelSurfaceVel = flywheelRps * 2.0 * Math.PI * config.flywheelRadiusMeters;
        double backRollerSurfaceVel = flywheelRps * config.backRollerGearRatio
            * 2.0 * Math.PI * config.backRollerRadiusMeters;
        return (flywheelSurfaceVel + backRollerSurfaceVel) / 2.0;
    }

    private static double calculateSpinRadPerSec(double flywheelRps, ShooterConfig config) {
        double flywheelSurfaceVel = flywheelRps * 2.0 * Math.PI * config.flywheelRadiusMeters;
        double backRollerSurfaceVel = flywheelRps * config.backRollerGearRatio
            * 2.0 * Math.PI * config.backRollerRadiusMeters;
        double deltaV = flywheelSurfaceVel - backRollerSurfaceVel;
        return deltaV / BallisticsConstants.BALL_RADIUS;
    }
}
