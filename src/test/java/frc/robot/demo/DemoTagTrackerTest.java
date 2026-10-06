package frc.robot.demo;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.Optional;
import java.util.TreeMap;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DemoTagTrackerTest {
    private static final int TAG = 26;
    private static final Pose3d TAG_IN_ROOM = new Pose3d(4.0, 0.0, 1.2, new Rotation3d(0.0, 0.0, Math.PI));

    private final TreeMap<Double, Pose2d> odometry = new TreeMap<>();
    private DemoTagTracker tracker;

    @BeforeEach
    void setUp() {
        odometry.clear();
        tracker = new DemoTagTracker(
            id -> id == TAG ? Optional.of(TagGeometryTest.OFFICIAL_TAG_26) : Optional.empty(),
            timestamp -> {
                var entry = odometry.floorEntry(timestamp + 1e-9);
                return entry == null ? Optional.empty() : Optional.of(entry.getValue());
            }
        );
    }

    private PoseObservation see(double timestamp, Pose2d robot, Pose3d tag) {
        odometry.put(timestamp, robot);
        return observation(timestamp, robot, tag, TAG, 1, 0.05, PoseObservationType.MEGATAG_1);
    }

    private static PoseObservation observation(
        double timestamp, Pose2d robot, Pose3d tag, int id, int tagCount, double ambiguity, PoseObservationType type
    ) {
        Pose3d reported = TagGeometryTest.reportedRobotPose(robot, tag);
        double distance = tag.getTranslation().getDistance(new Pose3d(robot).getTranslation());
        return new PoseObservation(timestamp, reported, ambiguity, tagCount, distance, type, id);
    }

    private static void assertNear(Translation3d expected, Translation3d actual, double tolerance) {
        assertEquals(expected.getX(), actual.getX(), tolerance);
        assertEquals(expected.getY(), actual.getY(), tolerance);
        assertEquals(expected.getZ(), actual.getZ(), tolerance);
    }

    @Test
    void acceptedFrame_placesTagInRoom() {
        assertEquals(DemoTagTracker.ObservationResult.ACCEPTED, tracker.addObservation(0, see(1.0, new Pose2d(), TAG_IN_ROOM)));

        assertTrue(tracker.isLocked(1.1));
        assertNear(TAG_IN_ROOM.getTranslation(), tracker.getLiveTarget(1.1).orElseThrow().position(), 1e-9);
        assertEquals(Math.PI, Math.abs(tracker.getFacingYawRadians().getAsDouble()), 1e-9);
    }

    @Test
    void movingRobot_tagStaysPutInRoom() {
        Pose2d[] path = {
            new Pose2d(0.0, 0.0, new Rotation2d()),
            new Pose2d(0.5, 0.3, Rotation2d.fromDegrees(20.0)),
            new Pose2d(1.0, -0.4, Rotation2d.fromDegrees(-35.0))
        };
        for (int i = 0; i < path.length; i++) {
            tracker.addObservation(i, see(1.0 + 0.05 * i, path[i], TAG_IN_ROOM));
        }

        assertNear(TAG_IN_ROOM.getTranslation(), tracker.getLiveTarget(1.2).orElseThrow().position(), 1e-9);
    }

    @Test
    void rejectsOtherTagsMultiTagMegaTag2AndFarFrames() {
        Pose2d robot = new Pose2d();
        odometry.put(1.0, robot);
        assertEquals(DemoTagTracker.ObservationResult.WRONG_ID,
            tracker.addObservation(0, observation(1.0, robot, TAG_IN_ROOM, 7, 1, 0.05, PoseObservationType.MEGATAG_1)));
        assertEquals(DemoTagTracker.ObservationResult.NO_SINGLE_TAG,
            tracker.addObservation(0, observation(1.0, robot, TAG_IN_ROOM, TAG, 2, 0.05, PoseObservationType.MEGATAG_1)));
        assertEquals(DemoTagTracker.ObservationResult.NOT_MEGATAG1,
            tracker.addObservation(0, observation(1.0, robot, TAG_IN_ROOM, TAG, 1, 0.05, PoseObservationType.MEGATAG_2)));
        Pose3d farTag = new Pose3d(30.0, 0.0, 1.2, new Rotation3d(0.0, 0.0, Math.PI));
        assertEquals(DemoTagTracker.ObservationResult.TOO_FAR,
            tracker.addObservation(0, observation(1.0, robot, farTag, TAG, 1, 0.05, PoseObservationType.MEGATAG_1)));

        assertFalse(tracker.isLiveTargetValid(1.0));
        assertEquals(4, tracker.getRejectedCount());
    }

    @Test
    void noOdometryAtFrameTime_rejected() {
        PoseObservation frame = observation(1.0, new Pose2d(), TAG_IN_ROOM, TAG, 1, 0.05, PoseObservationType.MEGATAG_1);
        assertEquals(DemoTagTracker.ObservationResult.NO_ODOMETRY, tracker.addObservation(0, frame));
    }

    @Test
    void medianIgnoresSingleOutlier() {
        Pose3d outlier = new Pose3d(6.0, 1.0, 2.0, TAG_IN_ROOM.getRotation());
        tracker.addObservation(0, see(1.00, new Pose2d(), TAG_IN_ROOM));
        tracker.addObservation(0, see(1.02, new Pose2d(), TAG_IN_ROOM));
        tracker.addObservation(0, see(1.04, new Pose2d(), outlier));
        tracker.addObservation(0, see(1.06, new Pose2d(), TAG_IN_ROOM));

        assertNear(TAG_IN_ROOM.getTranslation(), tracker.getLiveTarget(1.1).orElseThrow().position(), 1e-9);
    }

    @Test
    void liveTargetExpiresButSavedTargetStays() {
        tracker.addObservation(0, see(1.0, new Pose2d(), TAG_IN_ROOM));
        assertTrue(tracker.saveLiveTarget(1.1));

        assertFalse(tracker.isLocked(1.0 + DemoConstants.TAG_LOCKED_TIMEOUT_SEC + 0.1));
        assertTrue(tracker.isLiveTargetValid(1.0 + DemoConstants.LIVE_TARGET_TIMEOUT_SEC - 0.1));
        assertFalse(tracker.isLiveTargetValid(1.0 + DemoConstants.LIVE_TARGET_TIMEOUT_SEC + 0.1));

        double later = 100.0;
        assertEquals("SAVED", tracker.getActiveTarget(later).orElseThrow().source());
        tracker.clearSavedTarget();
        assertTrue(tracker.getActiveTarget(later).isEmpty());
    }

    @Test
    void saveFailsWhenTagNotLocked() {
        assertFalse(tracker.saveLiveTarget(1.0));
        tracker.addObservation(0, see(1.0, new Pose2d(), TAG_IN_ROOM));
        assertFalse(tracker.saveLiveTarget(3.0));
    }

    @Test
    void savedTargetWinsOverLiveTag() {
        tracker.addObservation(0, see(1.0, new Pose2d(), TAG_IN_ROOM));
        tracker.saveLiveTarget(1.0);
        Pose3d moved = new Pose3d(2.0, 2.0, 1.0, TAG_IN_ROOM.getRotation());
        tracker.addObservation(0, see(2.0, new Pose2d(), moved));

        assertNear(TAG_IN_ROOM.getTranslation(), tracker.getActiveTarget(2.0).orElseThrow().position(), 1e-9);
        assertNear(moved.getTranslation(), tracker.getLiveTarget(2.0).orElseThrow().position(), 1e-9);
    }

    @Test
    void placeTarget_aheadOfRobotAndFacingIt() {
        tracker.placeTarget(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(90.0)), 3.0, 0.0, 1.5);
        DemoTagTracker.TargetBase placed = tracker.getActiveTarget(0.0).orElseThrow();

        assertNear(new Translation3d(1.0, 4.0, 1.5), placed.position(), 1e-9);
        assertEquals(-Math.PI / 2.0, placed.facingYawRadians().getAsDouble(), 1e-9);
        assertEquals("PLACED", placed.source());
    }

    @Test
    void poseReset_keepsSavedTargetAtSameSpotRelativeToRobot() {
        Pose2d before = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0));
        tracker.placeTarget(before, 3.0, 0.0, 1.2);
        Pose2d after = new Pose2d(0.0, 0.0, new Rotation2d());

        tracker.onPoseReset(before, after);

        assertNear(new Translation3d(3.0, 0.0, 1.2), tracker.getSavedTarget().orElseThrow().position(), 1e-9);
    }

    @Test
    void ambiguousFrame_givesPositionButNoFacing() {
        Pose2d robot = new Pose2d();
        odometry.put(1.0, robot);
        tracker.addObservation(0, observation(1.0, robot, TAG_IN_ROOM, TAG, 1, 0.6, PoseObservationType.MEGATAG_1));

        assertTrue(tracker.isLiveTargetValid(1.0));
        assertFalse(tracker.getFacingYawRadians().isPresent());
    }

    @Test
    void changingTagId_dropsLiveTarget() {
        tracker.addObservation(0, see(1.0, new Pose2d(), TAG_IN_ROOM));
        tracker.setTagId(7);

        assertFalse(tracker.isLiveTargetValid(1.0));
    }

    @Test
    void median_handlesEvenAndOddCounts() {
        assertEquals(2.0, DemoTagTracker.median(new double[] {3.0, 1.0, 2.0}), 1e-12);
        assertEquals(2.5, DemoTagTracker.median(new double[] {4.0, 1.0, 2.0, 3.0}), 1e-12);
    }
}
