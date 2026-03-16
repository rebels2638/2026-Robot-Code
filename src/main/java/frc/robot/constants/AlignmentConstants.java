package frc.robot.constants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.ZoneConstants.RectangleZone;
import frc.robot.lib.util.ZoneUtil;

public final class AlignmentConstants {
    private AlignmentConstants() {}
    
    public static final class Tower {
        private Tower() {}

        public static final double INTERMEDIARY_MAX_VELOCITY_METERS_PER_SEC = 3;
        public static final double APPROACH_MAX_VELOCITY_METERS_PER_SEC = .7;

        public static final double INTERMEDIARY_TRANSLATION_TOLERANCE_METERS = 0.08;
        public static final double INTERMEDIARY_ROTATION_TOLERANCE_DEG = 5;

        public static final class Top {
            private Top() {}

            public static final Pair<Translation2d, Translation2d> BOUNDS = new Pair<>(
                ZoneConstants.Tower.TOP.cornerA(),
                ZoneConstants.Tower.TOP.cornerB()
            );

            public static final Pose2d INTERMEDIATE_WAYPOINT = new Pose2d(
                1.010, 
                4.911, 
                Rotation2d.fromDegrees(0)
            ); 

            public static final Pose2d FINAL_WAYPOINT = new Pose2d(
                1.010, 
                4.748, 
                Rotation2d.fromDegrees(0)
            );
        }
        public static final class Bottom {
            private Bottom() {}

            public static final Pair<Translation2d, Translation2d> BOUNDS = new Pair<>(
                ZoneConstants.Tower.BOTTOM.cornerA(),
                ZoneConstants.Tower.BOTTOM.cornerB()
            );

            public static final Pose2d INTERMEDIATE_WAYPOINT = new Pose2d(
                1.010, 
                2.434, 
                Rotation2d.fromDegrees(180)
            );

            public static final Pose2d FINAL_WAYPOINT = new Pose2d(
                1.010, 
                2.734, 
                Rotation2d.fromDegrees(180)
            );
        }

        public static boolean isWithinBounds(Pose2d robotPose, Pair<Translation2d, Translation2d> bounds) {
            RectangleZone zone = new RectangleZone(
                "alignment_bounds",
                bounds.getFirst(),
                bounds.getSecond()
            );
            boolean isWithinBounds = ZoneUtil.isPoseInZone(robotPose, zone, true);

            Logger.recordOutput("AlignmentConstants/isWithinBounds", isWithinBounds);
            if (Constants.VERBOSE_LOGGING_ENABLED) {
                Logger.recordOutput("AlignmentConstants/robotPose", robotPose);
            }
            return isWithinBounds;
        }
    }
}
