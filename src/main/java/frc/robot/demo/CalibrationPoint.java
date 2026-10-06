package frc.robot.demo;

/**
 * One calibration shot that hit the target: where the target was and the hood/flywheel values that
 * hit it. Values are absolute; corrections are derived against the model when the table loads.
 */
public record CalibrationPoint(
    double distanceMeters,
    double targetHeightMeters,
    double hoodDegrees,
    double flywheelRps,
    String savedAt
) {
    public boolean isValid() {
        return Double.isFinite(distanceMeters)
            && Double.isFinite(targetHeightMeters)
            && Double.isFinite(hoodDegrees)
            && Double.isFinite(flywheelRps)
            && distanceMeters > 0.0;
    }

    public String describe() {
        return String.format("%.2f m, %.2f m high -> %.1f deg, %.1f RPS", distanceMeters, targetHeightMeters, hoodDegrees, flywheelRps);
    }
}
