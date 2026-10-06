package frc.robot.demo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.OptionalDouble;

/** Geometry helpers for turning Limelight MegaTag1 poses into a tag position relative to the robot. */
public final class TagGeometry {
    private TagGeometry() {}

    /**
     * Tag pose relative to the robot.
     *
     * <p>The Limelight reports the robot pose as if the seen tag sat at its official field pose. Undoing
     * that against the same official pose leaves robot-to-camera times camera-to-tag, which is the
     * physical tag's 3D pose relative to the robot regardless of where or how tilted the tag really is.
     * Done in full 3D on purpose: flattening to (x, y, yaw) would be wrong for a tilted tag.
     */
    public static Transform3d robotToTag(Pose3d megaTagRobotPose, Pose3d officialTagPose) {
        return new Transform3d(megaTagRobotPose, officialTagPose);
    }

    /** Places a robot-relative tag into the room (odometry) frame. The robot frame origin is on the floor. */
    public static Pose3d tagInRoom(Pose2d odometryPose, Transform3d robotToTag) {
        return new Pose3d(odometryPose).transformBy(robotToTag);
    }

    /** Unit vector out of the printed face of the tag (WPILib tag frame +X). */
    public static Translation3d faceNormal(Rotation3d tagRotation) {
        return new Translation3d(1.0, 0.0, 0.0).rotateBy(tagRotation);
    }

    /** Direction the tag faces, flattened onto the floor. Empty when the tag is close to lying flat. */
    public static OptionalDouble facingYawRadians(Rotation3d tagRotation, double minHorizontalComponent) {
        Translation3d normal = faceNormal(tagRotation);
        double horizontal = Math.hypot(normal.getX(), normal.getY());
        if (horizontal < minHorizontalComponent) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(Math.atan2(normal.getY(), normal.getX()));
    }

    /** How far the face is tipped from vertical; positive means the face points upward. */
    public static double tiltDegrees(Rotation3d tagRotation) {
        Translation3d normal = faceNormal(tagRotation);
        return Math.toDegrees(Math.asin(Math.max(-1.0, Math.min(1.0, normal.getZ()))));
    }

    /**
     * Applies tag-relative offsets to a target.
     *
     * @param facingYawRadians direction out of the tag face toward the viewer
     * @param offsetOutMeters positive moves the aim point out of the tag face toward the viewer
     * @param offsetLeftMeters positive moves it left as seen by someone looking at the tag
     * @param aimHeightMeters absolute aim height above the floor
     */
    public static Translation3d applyOffsets(
        Translation2d basePosition,
        double facingYawRadians,
        double offsetOutMeters,
        double offsetLeftMeters,
        double aimHeightMeters
    ) {
        double fx = Math.cos(facingYawRadians);
        double fy = Math.sin(facingYawRadians);
        // A viewer looks along -f; their left is -f rotated +90 degrees.
        double leftX = fy;
        double leftY = -fx;
        return new Translation3d(
            basePosition.getX() + offsetOutMeters * fx + offsetLeftMeters * leftX,
            basePosition.getY() + offsetOutMeters * fy + offsetLeftMeters * leftY,
            aimHeightMeters
        );
    }

    /** Yaw pointing from a target toward the robot, used when the tag's facing is unknown. */
    public static double yawTowardRobot(Translation2d target, Translation2d robot) {
        Translation2d delta = robot.minus(target);
        if (delta.getNorm() < 1e-6) {
            return 0.0;
        }
        return Math.atan2(delta.getY(), delta.getX());
    }

    /** Re-expresses a room point after a pose reset so it keeps the same spot relative to the robot. */
    public static Translation2d reanchor(Translation2d point, Pose2d before, Pose2d after) {
        Translation2d robotRelative = point.minus(before.getTranslation()).rotateBy(before.getRotation().unaryMinus());
        return after.getTranslation().plus(robotRelative.rotateBy(after.getRotation()));
    }

    public static double reanchorYaw(double yawRadians, Pose2d before, Pose2d after) {
        Rotation2d delta = after.getRotation().minus(before.getRotation());
        return Rotation2d.fromRadians(yawRadians).plus(delta).getRadians();
    }
}
