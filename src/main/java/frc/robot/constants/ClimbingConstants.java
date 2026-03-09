package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.ZoneConstants.RectangleZone;
import java.util.List;
import java.util.Objects;

public final class ClimbingConstants {
    private ClimbingConstants() {}

    public static final double EXCLUSION_ZONE_PADDING_METERS = 0.05;
    public static final double ROTATION_CLEARANCE_DISTANCE_METERS = 0.6;
    public static final double ROTATION_CLEARANCE_HEADING_ERROR_DEG = 35.0;
    public static final double ROTATION_CLEARANCE_RETREAT_DISTANCE_METERS = 0.35;
    public static final double APPROACH_MAX_ACCELERATION_METERS_PER_SEC2 = 2.5;
    public static final double FINAL_APPROACH_MAX_ACCELERATION_METERS_PER_SEC2 = 1.5;

    public record AutoClimbTarget(
        String name,
        Pose2d preClimbPose,
        Pose2d finalClimbPose,
        List<RectangleZone> inclusionZones,
        List<RectangleZone> exclusionZones
    ) {
        public AutoClimbTarget {
            Objects.requireNonNull(name);
            Objects.requireNonNull(preClimbPose);
            Objects.requireNonNull(finalClimbPose);
            Objects.requireNonNull(inclusionZones);
            Objects.requireNonNull(exclusionZones);
            inclusionZones = List.copyOf(inclusionZones);
            exclusionZones = List.copyOf(exclusionZones);
        }
    }

    public static final AutoClimbTarget TOWER_LEFT = new AutoClimbTarget(
        "tower_left",
        AlignmentConstants.Tower.Left.INTERMEDIATE_WAYPOINT,
        AlignmentConstants.Tower.Left.FINAL_WAYPOINT,
        List.of(ZoneConstants.Tower.LEFT),
        List.of(ZoneConstants.Tower.EXCLUSION, ZoneConstants.Hub.EXCLUSION)
    );

    public static final AutoClimbTarget DEFAULT_TARGET = TOWER_LEFT;
}
