package frc.robot.demo;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class TagGeometryTest {
    // Official field pose of tag 26 (blue hub, facing the blue wall).
    static final Pose3d OFFICIAL_TAG_26 = new Pose3d(4.0218614, 4.0346376, 1.12395, new Rotation3d(0.0, 0.0, Math.PI));

    /** What a Limelight reports (MegaTag1 robot pose) when the real tag is somewhere else entirely. */
    static Pose3d reportedRobotPose(Pose2d robotInRoom, Pose3d physicalTagInRoom) {
        Transform3d robotToTag = new Transform3d(new Pose3d(robotInRoom), physicalTagInRoom);
        return OFFICIAL_TAG_26.transformBy(robotToTag.inverse());
    }

    private static void assertTranslationEquals(Translation3d expected, Translation3d actual) {
        assertEquals(expected.getX(), actual.getX(), 1e-9);
        assertEquals(expected.getY(), actual.getY(), 1e-9);
        assertEquals(expected.getZ(), actual.getZ(), 1e-9);
    }

    @Test
    void robotToTag_recoversPhysicalTagEvenWhenTilted() {
        Pose2d robot = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
        // Held at an angle: yawed, leaned back 30 deg and rotated in its own plane.
        Pose3d physicalTag = new Pose3d(4.0, 3.0, 1.5, new Rotation3d(
            Math.toRadians(15.0), Math.toRadians(-30.0), Math.toRadians(200.0)));

        Transform3d robotToTag = TagGeometry.robotToTag(reportedRobotPose(robot, physicalTag), OFFICIAL_TAG_26);
        Pose3d recovered = TagGeometry.tagInRoom(robot, robotToTag);

        assertTranslationEquals(physicalTag.getTranslation(), recovered.getTranslation());
        assertEquals(0.0, recovered.getRotation().minus(physicalTag.getRotation()).getAngle(), 1e-9);
    }

    @Test
    void reportedPoseOfTiltedTag_isNotFlat_soTwoDimensionalMathWouldBeWrong() {
        Pose2d robot = new Pose2d(0.0, 0.0, new Rotation2d());
        Pose3d leaningTag = new Pose3d(3.0, 0.0, 1.2, new Rotation3d(0.0, Math.toRadians(-30.0), Math.PI));

        Pose3d reported = reportedRobotPose(robot, leaningTag);

        assertTrue(Math.abs(reported.getRotation().getY()) > Math.toRadians(20.0));
    }

    @Test
    void facingYaw_verticalTag() {
        assertEquals(Math.PI, Math.abs(TagGeometry.facingYawRadians(new Rotation3d(0.0, 0.0, Math.PI), 0.3).getAsDouble()), 1e-9);
    }

    @Test
    void facingYaw_flatTagHasNoFacing() {
        // Pitch -90 turns the face straight up.
        assertFalse(TagGeometry.facingYawRadians(new Rotation3d(0.0, Math.toRadians(-90.0), 0.0), 0.3).isPresent());
    }

    @Test
    void tilt_positiveWhenFaceTipsUp() {
        assertEquals(30.0, TagGeometry.tiltDegrees(new Rotation3d(0.0, Math.toRadians(-30.0), 0.0)), 1e-9);
        assertEquals(0.0, TagGeometry.tiltDegrees(new Rotation3d(0.0, 0.0, 1.0)), 1e-9);
    }

    @Test
    void applyOffsets_outIsTowardViewer_leftIsViewersLeft() {
        Translation2d base = new Translation2d(5.0, 2.0);
        // Tag faces +X, so the viewer stands at +X looking toward -X; their left is -Y.
        Translation3d moved = TagGeometry.applyOffsets(base, 0.0, 1.0, 0.5, 1.7);

        assertEquals(6.0, moved.getX(), 1e-9);
        assertEquals(1.5, moved.getY(), 1e-9);
        assertEquals(1.7, moved.getZ(), 1e-9);
    }

    @Test
    void reanchor_keepsPointFixedRelativeToRobot() {
        Pose2d before = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(90.0));
        Pose2d after = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(0.0));
        // Two meters straight ahead of the robot before the reset...
        Translation2d moved = TagGeometry.reanchor(new Translation2d(2.0, 3.0), before, after);

        // ...is still two meters straight ahead after it.
        assertEquals(4.0, moved.getX(), 1e-9);
        assertEquals(1.0, moved.getY(), 1e-9);
        assertEquals(0.0, TagGeometry.reanchorYaw(Math.PI / 2.0, before, after), 1e-9);
    }
}
