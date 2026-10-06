package frc.robot.demo;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class GentleShotModelTest {
    private static final double SHOOTER_Z = 0.69;
    private static final double HUB_Z = 72.0 * 0.0254;

    private static double heightAt(double distance, double speed, double thetaRad) {
        return distance * Math.tan(thetaRad)
            - GentleShotModel.GRAVITY * distance * distance / (2.0 * speed * speed * Math.cos(thetaRad) * Math.cos(thetaRad));
    }

    @Test
    void solve_threeMetersToChestHeightTag_matchesWorkedExample() {
        GentleShotModel.Solution solution = GentleShotModel.solve(3.0, 1.2 - SHOOTER_Z, 40.0, 70.0);

        assertTrue(solution.feasible());
        assertEquals(49.82, solution.hoodDegrees(), 0.05);
        assertEquals(5.90, solution.launchSpeedMetersPerSec(), 0.02);
        assertEquals(26.1, solution.flywheelRps(), 0.15);
    }

    @Test
    void solve_pathPassesThroughTarget() {
        double[][] cases = {{2.0, 0.51}, {4.0, -0.69}, {3.0, 1.51}, {6.0, 0.3}};
        for (double[] testCase : cases) {
            GentleShotModel.Solution solution = GentleShotModel.solve(testCase[0], testCase[1], 40.0, 70.0);
            assertTrue(solution.feasible());
            double height = heightAt(testCase[0], solution.launchSpeedMetersPerSec(), Math.toRadians(solution.hoodDegrees()));
            assertEquals(testCase[1], height, 1e-9);
        }
    }

    @Test
    void gentleAngle_needsLessSpeedThanSteeperOrFlatterAngles() {
        double distance = 3.0;
        double height = 0.51;
        double gentle = GentleShotModel.gentleLaunchAngleDegrees(distance, height, 0.0, 90.0);
        double gentleSpeed = GentleShotModel.requiredLaunchSpeed(distance, height, Math.toRadians(gentle));
        assertTrue(GentleShotModel.requiredLaunchSpeed(distance, height, Math.toRadians(gentle + 8.0)) > gentleSpeed);
        assertTrue(GentleShotModel.requiredLaunchSpeed(distance, height, Math.toRadians(gentle - 8.0)) > gentleSpeed);
    }

    @Test
    void gentleAngle_clampsToHoodRange() {
        // A floor target wants a flatter angle than the hood allows.
        assertEquals(40.0, GentleShotModel.gentleLaunchAngleDegrees(2.0, -0.69, 40.0, 70.0), 1e-9);
        assertEquals(70.0, GentleShotModel.gentleLaunchAngleDegrees(0.2, 3.0, 40.0, 70.0), 1e-9);
    }

    @Test
    void solve_targetTooHighForMaxHood_isInfeasible() {
        assertFalse(GentleShotModel.solve(0.5, 2.0, 40.0, 70.0).feasible());
        assertFalse(GentleShotModel.solve(0.0, 0.5, 40.0, 70.0).feasible());
    }

    @Test
    void exitSpeedAndRps_areInverses() {
        for (double speed = 3.0; speed <= 12.0; speed += 1.5) {
            assertEquals(speed, GentleShotModel.exitSpeedForRps(GentleShotModel.rpsForExitSpeed(speed)), 1e-9);
        }
        assertEquals(0.0, GentleShotModel.exitSpeedForRps(0.0), 1e-9);
    }

    @Test
    void fitConstants_reproduceCompTableAnchorsWithinFivePercent() {
        // The 13 hand-tuned comp rows: {distance, hood, rps, target height}.
        double[][] anchors = {
            {1.4, 70, 25.4, HUB_Z}, {2.2, 70, 27.4, HUB_Z}, {3.15, 65, 30.4, HUB_Z}, {3.72, 60, 30.4, HUB_Z},
            {5.05, 58, 34.4, HUB_Z}, {5.5, 57, 34.7, HUB_Z}, {6.0, 57, 38.0, HUB_Z}, {6.5, 57, 40.7, HUB_Z},
            {0.6, 40, 8.0, 0.0}, {1.3, 40, 13.0, 0.0}, {2.3, 40, 17.5, 0.0}, {3.5, 40, 26.0, 0.0}, {6.15, 40, 33.0, 0.0}
        };
        double sumSquares = 0.0;
        for (double[] anchor : anchors) {
            double predicted = GentleShotModel.solveAtAngle(anchor[0], anchor[3] - SHOOTER_Z, anchor[1]).flywheelRps();
            double relativeError = predicted / anchor[2] - 1.0;
            sumSquares += relativeError * relativeError;
            assertTrue(Math.abs(relativeError) < 0.10, "row " + anchor[0] + " m off by " + relativeError);
        }
        assertTrue(Math.sqrt(sumSquares / anchors.length) < 0.05);
    }

    @Test
    void predict_stationaryRobot_usesEnergyConservation() {
        double rps = 26.7;
        double hood = 49.8;
        double speed = GentleShotModel.exitSpeedForRps(rps);
        GentleShotModel.GroundPrediction prediction = GentleShotModel.predict(hood, rps, 0.0, 0.0, 0.0, 0.51);

        assertEquals(Math.sqrt(speed * speed - 2.0 * GentleShotModel.GRAVITY * 0.51), prediction.speedAtTargetMetersPerSec(), 1e-9);
        double vz = speed * Math.sin(Math.toRadians(hood));
        assertEquals(vz * vz / (2.0 * GentleShotModel.GRAVITY), prediction.peakHeightAboveShooterMeters(), 1e-9);
    }

    @Test
    void predict_drivingTowardTarget_hitsHarder() {
        GentleShotModel.GroundPrediction still = GentleShotModel.predict(50.0, 26.0, 0.0, 0.0, 0.0, 0.5);
        GentleShotModel.GroundPrediction toward = GentleShotModel.predict(50.0, 26.0, 0.0, 1.0, 0.0, 0.5);
        GentleShotModel.GroundPrediction away = GentleShotModel.predict(50.0, 26.0, 0.0, -1.0, 0.0, 0.5);

        assertTrue(toward.speedAtTargetMetersPerSec() > still.speedAtTargetMetersPerSec());
        assertTrue(away.speedAtTargetMetersPerSec() < still.speedAtTargetMetersPerSec());
        assertEquals(still.peakHeightAboveShooterMeters(), toward.peakHeightAboveShooterMeters(), 1e-12);
    }
}
