package frc.robot.demo;

import java.util.ArrayList;
import java.util.List;

/**
 * Blends calibration points into hood/flywheel corrections for any (distance, target height).
 *
 * <p>Each point's correction is "what hit minus what the model said" at that point. Corrections
 * blend by closeness (inverse distance squared) and fade out away from every point: full strength
 * within 0.5 m of distance (0.25 m of height) of the nearest point, gone by 1.5 m (0.75 m of height).
 */
public final class ShotCorrectionTable {
    public static final double DISTANCE_SCALE_METERS = 1.0;
    public static final double HEIGHT_SCALE_METERS = 0.5;
    public static final double FULL_STRENGTH_RADIUS = 0.5;
    public static final double FADE_OUT_RADIUS = 1.5;
    private static final double EXACT_MATCH_RADIUS = 1e-9;

    /** Model hood/RPS for a target, used to turn absolute calibration values into corrections. */
    @FunctionalInterface
    public interface Model {
        GentleShotModel.Solution solve(double distanceMeters, double targetHeightMeters);
    }

    public record Correction(double hoodDegrees, double flywheelRps, double strength) {
        public static final Correction NONE = new Correction(0.0, 0.0, 0.0);
    }

    private record Offset(CalibrationPoint point, double hoodDegrees, double flywheelRps) {}

    private final List<CalibrationPoint> points;
    private final List<Offset> offsets = new ArrayList<>();

    public ShotCorrectionTable(List<CalibrationPoint> points, Model model) {
        this.points = List.copyOf(points);
        for (CalibrationPoint point : this.points) {
            if (!point.isValid()) {
                continue;
            }
            GentleShotModel.Solution modelSolution = model.solve(point.distanceMeters(), point.targetHeightMeters());
            if (!modelSolution.feasible()) {
                continue;
            }
            offsets.add(new Offset(
                point,
                point.hoodDegrees() - modelSolution.hoodDegrees(),
                point.flywheelRps() - modelSolution.flywheelRps()
            ));
        }
    }

    public List<CalibrationPoint> points() {
        return points;
    }

    public int usablePointCount() {
        return offsets.size();
    }

    public Correction correctionAt(double distanceMeters, double targetHeightMeters) {
        double weightSum = 0.0;
        double hoodSum = 0.0;
        double rpsSum = 0.0;
        double nearestRadius = Double.POSITIVE_INFINITY;

        for (Offset offset : offsets) {
            double radius = normalizedDistance(
                distanceMeters,
                targetHeightMeters,
                offset.point().distanceMeters(),
                offset.point().targetHeightMeters()
            );
            if (radius <= EXACT_MATCH_RADIUS) {
                return new Correction(offset.hoodDegrees(), offset.flywheelRps(), 1.0);
            }
            if (radius >= FADE_OUT_RADIUS) {
                continue;
            }
            nearestRadius = Math.min(nearestRadius, radius);
            double weight = 1.0 / (radius * radius);
            weightSum += weight;
            hoodSum += weight * offset.hoodDegrees();
            rpsSum += weight * offset.flywheelRps();
        }

        if (weightSum <= 0.0) {
            return Correction.NONE;
        }
        double strength = fadeStrength(nearestRadius);
        return new Correction(strength * hoodSum / weightSum, strength * rpsSum / weightSum, strength);
    }

    static double normalizedDistance(double distanceA, double heightA, double distanceB, double heightB) {
        double distanceTerm = (distanceA - distanceB) / DISTANCE_SCALE_METERS;
        double heightTerm = (heightA - heightB) / HEIGHT_SCALE_METERS;
        return Math.hypot(distanceTerm, heightTerm);
    }

    static double fadeStrength(double nearestRadius) {
        if (nearestRadius <= FULL_STRENGTH_RADIUS) {
            return 1.0;
        }
        if (nearestRadius >= FADE_OUT_RADIUS) {
            return 0.0;
        }
        return (FADE_OUT_RADIUS - nearestRadius) / (FADE_OUT_RADIUS - FULL_STRENGTH_RADIUS);
    }
}
