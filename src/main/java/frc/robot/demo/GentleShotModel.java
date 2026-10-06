package frc.robot.demo;

import edu.wpi.first.math.MathUtil;

/**
 * Shot model for the demo's tag mode.
 *
 * <p>Launch angle is the "gentle" (minimum launch speed) angle for the target, so the ball also
 * arrives as slowly as physics allows. Launch speed comes from the drag-free projectile equation.
 * Speed is converted to flywheel RPS with a linear fit to the 13 hand-tuned rows of the comp
 * shooting and pass lerp tables (comp shooter, hood angle treated as launch angle; RMS fit error
 * about 4% of RPS). Interpolated and extrapolated table rows were excluded from the fit.
 */
public final class GentleShotModel {
    public static final double GRAVITY = 9.81;
    /** Ball exit speed gained per flywheel RPS, fit to the comp lerp table anchors. */
    public static final double EXIT_SPEED_PER_RPS = 0.2369;
    /** Ball exit speed at zero RPS from the same fit (small slip/offset term). */
    public static final double EXIT_SPEED_OFFSET_MPS = -0.2895;

    private GentleShotModel() {}

    public record Solution(
        boolean feasible,
        double hoodDegrees,
        double launchSpeedMetersPerSec,
        double flywheelRps,
        double flightTimeSeconds
    ) {
        static Solution infeasible(double hoodDegrees) {
            return new Solution(false, hoodDegrees, Double.NaN, Double.NaN, Double.NaN);
        }
    }

    /** Ball speed relative to the ground and the highest point of the arc. */
    public record GroundPrediction(double speedAtTargetMetersPerSec, double peakHeightAboveShooterMeters) {}

    public static double exitSpeedForRps(double flywheelRps) {
        return Math.max(0.0, EXIT_SPEED_PER_RPS * flywheelRps + EXIT_SPEED_OFFSET_MPS);
    }

    public static double rpsForExitSpeed(double exitSpeedMetersPerSec) {
        return (exitSpeedMetersPerSec - EXIT_SPEED_OFFSET_MPS) / EXIT_SPEED_PER_RPS;
    }

    /** Minimum-launch-speed angle: 45 deg plus half the elevation angle to the target. */
    public static double gentleLaunchAngleDegrees(
        double horizontalDistanceMeters,
        double heightAboveShooterMeters,
        double minHoodDegrees,
        double maxHoodDegrees
    ) {
        double elevationDegrees = Math.toDegrees(Math.atan2(heightAboveShooterMeters, horizontalDistanceMeters));
        return MathUtil.clamp(45.0 + 0.5 * elevationDegrees, minHoodDegrees, maxHoodDegrees);
    }

    /** Drag-free launch speed whose path passes through the target, or NaN if no such speed exists. */
    public static double requiredLaunchSpeed(
        double horizontalDistanceMeters,
        double heightAboveShooterMeters,
        double launchAngleRadians
    ) {
        if (!(horizontalDistanceMeters > 0.0)) {
            return Double.NaN;
        }
        double cos = Math.cos(launchAngleRadians);
        double clearance = horizontalDistanceMeters * Math.tan(launchAngleRadians) - heightAboveShooterMeters;
        if (!(clearance > 0.0) || !(cos > 0.0)) {
            return Double.NaN;
        }
        return Math.sqrt(GRAVITY * horizontalDistanceMeters * horizontalDistanceMeters / (2.0 * cos * cos * clearance));
    }

    public static Solution solve(
        double horizontalDistanceMeters,
        double heightAboveShooterMeters,
        double minHoodDegrees,
        double maxHoodDegrees
    ) {
        double hoodDegrees = gentleLaunchAngleDegrees(
            horizontalDistanceMeters,
            heightAboveShooterMeters,
            minHoodDegrees,
            maxHoodDegrees
        );
        return solveAtAngle(horizontalDistanceMeters, heightAboveShooterMeters, hoodDegrees);
    }

    public static Solution solveAtAngle(
        double horizontalDistanceMeters,
        double heightAboveShooterMeters,
        double hoodDegrees
    ) {
        double theta = Math.toRadians(hoodDegrees);
        double launchSpeed = requiredLaunchSpeed(horizontalDistanceMeters, heightAboveShooterMeters, theta);
        if (!Double.isFinite(launchSpeed)) {
            return Solution.infeasible(hoodDegrees);
        }
        return new Solution(
            true,
            hoodDegrees,
            launchSpeed,
            rpsForExitSpeed(launchSpeed),
            flightTimeSeconds(horizontalDistanceMeters, launchSpeed, theta)
        );
    }

    public static double flightTimeSeconds(double horizontalDistanceMeters, double launchSpeedMetersPerSec, double launchAngleRadians) {
        double horizontalSpeed = launchSpeedMetersPerSec * Math.cos(launchAngleRadians);
        return horizontalSpeed > 1e-6 ? horizontalDistanceMeters / horizontalSpeed : Double.NaN;
    }

    /**
     * Predicts how fast the ball is moving (relative to a stationary target) when it reaches the
     * target height, and how high the arc peaks. Robot motion adds to the ball's ground speed.
     */
    public static GroundPrediction predict(
        double hoodDegrees,
        double flywheelRps,
        double azimuthRadians,
        double shooterVxFieldMetersPerSec,
        double shooterVyFieldMetersPerSec,
        double targetHeightAboveShooterMeters
    ) {
        double launchSpeed = exitSpeedForRps(flywheelRps);
        double theta = Math.toRadians(hoodDegrees);
        double horizontal = launchSpeed * Math.cos(theta);
        double vx = horizontal * Math.cos(azimuthRadians) + shooterVxFieldMetersPerSec;
        double vy = horizontal * Math.sin(azimuthRadians) + shooterVyFieldMetersPerSec;
        double vz = launchSpeed * Math.sin(theta);
        double speedSquaredAtTarget = vx * vx + vy * vy + vz * vz - 2.0 * GRAVITY * targetHeightAboveShooterMeters;
        double peak = vz > 0.0 ? vz * vz / (2.0 * GRAVITY) : 0.0;
        return new GroundPrediction(Math.sqrt(Math.max(0.0, speedSquaredAtTarget)), peak);
    }
}
