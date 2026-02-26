package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.gson.Gson;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.configs.ShooterConfig;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.util.ballistics.BallisticsConstants;
import frc.robot.lib.util.ballistics.BallisticsPhysics;
import frc.robot.lib.util.ballistics.TrajectoryResult;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class ShotLerpDiagnosticsTest {
    @Test
    // Regression: solved setpoint exit velocity should reproduce lerp-table flight time under the same ballistics model.
    void simLerpTable_currentMethodMatchesConfiguredFlightTimes() throws IOException {
        ShooterConfig config = loadShooterConfig("src/main/deploy/configs/shooter/sim.json");
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable = config.getLerpTable();

        double targetHeight = FieldConstants.Hub.hubCenter.getZ();
        double meanAbsFlightTimeErrorSeconds = 0.0;

        for (ShooterConfig.LerpEntry entry : config.lerpTable) {
            double angleRad = Math.toRadians(entry.hoodAngleDegrees);
            double rps = entry.flywheelVelocityRPS;
            double spinRateRadPerSec = ShotCalculator.calculateSpinRateRadPerSec(
                entry.distanceMeters,
                lerpTable,
                rps,
                measuredRps -> calculateShotExitVelocityMetersPerSec(measuredRps, config),
                measuredRps -> calculateSpinRadPerSec(measuredRps, config),
                config.shooterPoseZ,
                targetHeight
            );

            double currentMethodExitVelocity = ShotCalculator.calculateExitVelocityMetersPerSec(
                entry.distanceMeters,
                lerpTable,
                rps,
                measuredRps -> calculateShotExitVelocityMetersPerSec(measuredRps, config),
                measuredRps -> calculateSpinRadPerSec(measuredRps, config),
                config.shooterPoseZ,
                targetHeight
            );

            TrajectoryResult currentResult = BallisticsPhysics.simulateToDistance(
                entry.distanceMeters,
                angleRad,
                currentMethodExitVelocity,
                config.shooterPoseZ,
                targetHeight,
                spinRateRadPerSec,
                0.002
            );

            meanAbsFlightTimeErrorSeconds += Math.abs(currentResult.flightTime() - entry.flightTimeSeconds);
        }

        meanAbsFlightTimeErrorSeconds /= config.lerpTable.size();
        assertTrue(meanAbsFlightTimeErrorSeconds < 0.02);
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
