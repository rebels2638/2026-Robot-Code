package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.ZoneConstants.RectangleZone;
import java.util.List;
import java.util.Objects;

public final class ClimbingConstants {
    private ClimbingConstants() {}

    public static final double ROTATION_CLEARANCE_DISTANCE_METERS = 0.6;
    public static final double ROTATION_CLEARANCE_HEADING_ERROR_DEG = 45.0;
    public static final double APPROACH_MAX_VELOCITY_METERS_PER_SEC = 2.5;
    public static final double FINAL_APPROACH_MAX_VELOCITY_METERS_PER_SEC = 1;
    public static final double APPROACH_MAX_ACCELERATION_METERS_PER_SEC2 = 12.0;
    public static final double FINAL_APPROACH_MAX_ACCELERATION_METERS_PER_SEC2 = 12.0;
    public static final double APPROACH_END_TRANSLATION_TOLERANCE_METERS = 0.05;
    public static final double FINAL_APPROACH_END_TRANSLATION_TOLERANCE_METERS = 0.05;
    public static final double APPROACH_END_ROTATION_TOLERANCE_DEG = 8.0;
    public static final double FINAL_APPROACH_END_ROTATION_TOLERANCE_DEG = 8.0;

    public record AutoClimbSide(
        String name,
        Pose2d preClimbPose,
        Pose2d finalClimbPose,
        List<RectangleZone> inclusionZones
    ) {
        public AutoClimbSide {
            Objects.requireNonNull(name);
            Objects.requireNonNull(preClimbPose);
            Objects.requireNonNull(finalClimbPose);
            Objects.requireNonNull(inclusionZones);
            inclusionZones = List.copyOf(inclusionZones);
        }
    }

    public record AutoClimbTarget(
        String name,
        List<AutoClimbSide> sides
    ) {
        public AutoClimbTarget {
            Objects.requireNonNull(name);
            Objects.requireNonNull(sides);
            sides = List.copyOf(sides);
        }
    }

    public static final AutoClimbSide TOWER_TOP = new AutoClimbSide(
        "tower_top",
        AlignmentConstants.Tower.Top.INTERMEDIATE_WAYPOINT,
        AlignmentConstants.Tower.Top.FINAL_WAYPOINT,
        List.of(ZoneConstants.Tower.TOP)
    );

    public static final AutoClimbSide TOWER_BOTTOM = new AutoClimbSide(
        "tower_bottom",
        AlignmentConstants.Tower.Bottom.INTERMEDIATE_WAYPOINT,
        AlignmentConstants.Tower.Bottom.FINAL_WAYPOINT,
        List.of(ZoneConstants.Tower.BOTTOM)
    );

    public static final AutoClimbTarget TOWER = new AutoClimbTarget(
        "tower",
        List.of(TOWER_TOP, TOWER_BOTTOM)
    );

    public static final AutoClimbTarget DEFAULT_TARGET = TOWER;
}
