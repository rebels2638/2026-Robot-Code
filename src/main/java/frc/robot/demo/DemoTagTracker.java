package frc.robot.demo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.DoubleFunction;
import java.util.function.IntFunction;

/**
 * Tracks the demo AprilTag as a point in the room.
 *
 * <p>Each accepted MegaTag1 frame gives the tag's pose relative to the robot; that is pinned into
 * the room with the robot's odometry pose at the frame's capture time, so robot motion does not
 * smear the estimate. The live estimate is a per-axis median over a short window. A saved target
 * freezes a copy that keeps being used after the tag is gone.
 */
public final class DemoTagTracker {
    public enum ObservationResult {
        ACCEPTED,
        NOT_MEGATAG1,
        NO_SINGLE_TAG,
        WRONG_ID,
        TOO_FAR,
        UNKNOWN_TAG,
        NO_ODOMETRY,
        INVALID
    }

    /** One accepted frame. Position and facing are in the room (odometry) frame. */
    public record Sample(
        double timestampSeconds,
        int cameraIndex,
        Translation3d position,
        OptionalDouble facingYawRadians,
        double tiltDegrees,
        double ambiguity,
        Transform3d robotToTag
    ) {}

    /** A target before offsets: tag center (or placed point) plus which way it faces, if known. */
    public record TargetBase(Translation3d position, OptionalDouble facingYawRadians, String source) {}

    private final IntFunction<Optional<Pose3d>> officialTagPoseLookup;
    private final DoubleFunction<Optional<Pose2d>> odometryPoseAtTime;
    private final ArrayDeque<Sample> window = new ArrayDeque<>();

    private int tagId = DemoConstants.DEFAULT_TAG_ID;
    private Translation3d liveEstimate = null;
    private double lastSeenTimestamp = Double.NEGATIVE_INFINITY;
    private OptionalDouble lastFacingYaw = OptionalDouble.empty();
    private Sample lastSample = null;
    private TargetBase savedTarget = null;
    private long acceptedCount = 0;
    private long rejectedCount = 0;
    private ObservationResult lastRejectReason = ObservationResult.ACCEPTED;

    public DemoTagTracker(
        IntFunction<Optional<Pose3d>> officialTagPoseLookup,
        DoubleFunction<Optional<Pose2d>> odometryPoseAtTime
    ) {
        this.officialTagPoseLookup = officialTagPoseLookup;
        this.odometryPoseAtTime = odometryPoseAtTime;
    }

    public int getTagId() {
        return tagId;
    }

    /** Switching tag IDs drops the live estimate; a saved target is kept. */
    public void setTagId(int newTagId) {
        if (newTagId == tagId) {
            return;
        }
        tagId = newTagId;
        window.clear();
        liveEstimate = null;
        lastSample = null;
        lastFacingYaw = OptionalDouble.empty();
        lastSeenTimestamp = Double.NEGATIVE_INFINITY;
    }

    public ObservationResult addObservation(int cameraIndex, PoseObservation observation) {
        ObservationResult result = classify(observation);
        if (result != ObservationResult.ACCEPTED) {
            return reject(result);
        }

        Optional<Pose3d> officialTagPose = officialTagPoseLookup.apply(tagId);
        if (officialTagPose.isEmpty()) {
            return reject(ObservationResult.UNKNOWN_TAG);
        }
        Optional<Pose2d> odometryPose = odometryPoseAtTime.apply(observation.timestamp());
        if (odometryPose.isEmpty()) {
            return reject(ObservationResult.NO_ODOMETRY);
        }

        Transform3d robotToTag = TagGeometry.robotToTag(observation.pose(), officialTagPose.get());
        Pose3d tagInRoom = TagGeometry.tagInRoom(odometryPose.get(), robotToTag);
        OptionalDouble facing = observation.ambiguity() <= DemoConstants.MAX_ORIENTATION_AMBIGUITY
            ? TagGeometry.facingYawRadians(tagInRoom.getRotation(), DemoConstants.MIN_FACING_HORIZONTAL_COMPONENT)
            : OptionalDouble.empty();

        Sample sample = new Sample(
            observation.timestamp(),
            cameraIndex,
            tagInRoom.getTranslation(),
            facing,
            TagGeometry.tiltDegrees(tagInRoom.getRotation()),
            observation.ambiguity(),
            robotToTag
        );
        window.addLast(sample);
        lastSample = sample;
        lastSeenTimestamp = Math.max(lastSeenTimestamp, observation.timestamp());
        acceptedCount++;
        pruneOlderThan(lastSeenTimestamp - DemoConstants.TAG_SMOOTHING_WINDOW_SEC);
        recomputeEstimate();
        return ObservationResult.ACCEPTED;
    }

    private ObservationResult classify(PoseObservation observation) {
        if (observation == null || observation.pose() == null || !Double.isFinite(observation.timestamp())) {
            return ObservationResult.INVALID;
        }
        if (observation.type() != PoseObservationType.MEGATAG_1) {
            return ObservationResult.NOT_MEGATAG1;
        }
        if (observation.tagCount() != 1) {
            return ObservationResult.NO_SINGLE_TAG;
        }
        if (observation.primaryTagId() != tagId) {
            return ObservationResult.WRONG_ID;
        }
        if (!(observation.averageTagDistance() <= DemoConstants.MAX_TAG_RANGE_METERS)) {
            return ObservationResult.TOO_FAR;
        }
        return ObservationResult.ACCEPTED;
    }

    private ObservationResult reject(ObservationResult reason) {
        rejectedCount++;
        lastRejectReason = reason;
        return reason;
    }

    /** Drops samples older than the smoothing window. Call once per loop. */
    public void update(double nowSeconds) {
        if (pruneOlderThan(nowSeconds - DemoConstants.TAG_SMOOTHING_WINDOW_SEC) && !window.isEmpty()) {
            recomputeEstimate();
        }
    }

    /** Removes samples captured before the cutoff. Returns true if any were removed. */
    private boolean pruneOlderThan(double cutoffSeconds) {
        boolean removed = false;
        while (!window.isEmpty() && window.peekFirst().timestampSeconds() < cutoffSeconds) {
            window.removeFirst();
            removed = true;
        }
        return removed;
    }

    private void recomputeEstimate() {
        int count = window.size();
        if (count == 0) {
            return;
        }
        double[] xs = new double[count];
        double[] ys = new double[count];
        double[] zs = new double[count];
        double sinSum = 0.0;
        double cosSum = 0.0;
        int facingCount = 0;
        int i = 0;
        for (Sample sample : window) {
            xs[i] = sample.position().getX();
            ys[i] = sample.position().getY();
            zs[i] = sample.position().getZ();
            i++;
            if (sample.facingYawRadians().isPresent()) {
                sinSum += Math.sin(sample.facingYawRadians().getAsDouble());
                cosSum += Math.cos(sample.facingYawRadians().getAsDouble());
                facingCount++;
            }
        }
        liveEstimate = new Translation3d(median(xs), median(ys), median(zs));
        if (facingCount > 0 && Math.hypot(sinSum, cosSum) > 1e-9) {
            lastFacingYaw = OptionalDouble.of(Math.atan2(sinSum, cosSum));
        }
    }

    static double median(double[] values) {
        double[] sorted = values.clone();
        Arrays.sort(sorted);
        int middle = sorted.length / 2;
        return sorted.length % 2 == 1 ? sorted[middle] : 0.5 * (sorted[middle - 1] + sorted[middle]);
    }

    public boolean isLocked(double nowSeconds) {
        return nowSeconds - lastSeenTimestamp <= DemoConstants.TAG_LOCKED_TIMEOUT_SEC;
    }

    public boolean isLiveTargetValid(double nowSeconds) {
        return liveEstimate != null && nowSeconds - lastSeenTimestamp <= DemoConstants.LIVE_TARGET_TIMEOUT_SEC;
    }

    public double getSecondsSinceSeen(double nowSeconds) {
        return nowSeconds - lastSeenTimestamp;
    }

    public Optional<TargetBase> getLiveTarget(double nowSeconds) {
        if (!isLiveTargetValid(nowSeconds)) {
            return Optional.empty();
        }
        return Optional.of(new TargetBase(liveEstimate, lastFacingYaw, "LIVE"));
    }

    /** Saved target if there is one, otherwise the live tag. */
    public Optional<TargetBase> getActiveTarget(double nowSeconds) {
        if (savedTarget != null) {
            return Optional.of(savedTarget);
        }
        return getLiveTarget(nowSeconds);
    }

    public Optional<TargetBase> getSavedTarget() {
        return Optional.ofNullable(savedTarget);
    }

    /** Saves the live tag if it is currently locked. Returns false when there is nothing fresh to save. */
    public boolean saveLiveTarget(double nowSeconds) {
        if (!isLocked(nowSeconds) || liveEstimate == null) {
            return false;
        }
        savedTarget = new TargetBase(liveEstimate, lastFacingYaw, "SAVED");
        return true;
    }

    /** Saves a hand-placed target relative to the robot. It faces the robot so offsets stay intuitive. */
    public void placeTarget(Pose2d robotPose, double forwardMeters, double leftMeters, double heightMeters) {
        Translation2d position = robotPose.getTranslation()
            .plus(new Translation2d(forwardMeters, leftMeters).rotateBy(robotPose.getRotation()));
        double facing = TagGeometry.yawTowardRobot(position, robotPose.getTranslation());
        savedTarget = new TargetBase(
            new Translation3d(position.getX(), position.getY(), heightMeters),
            OptionalDouble.of(facing),
            "PLACED"
        );
    }

    public void clearSavedTarget() {
        savedTarget = null;
    }

    /** Keeps stored points at the same spot in the room after the robot's pose is reset (e.g. zero gyro). */
    public void onPoseReset(Pose2d before, Pose2d after) {
        if (savedTarget != null) {
            savedTarget = reanchor(savedTarget, before, after);
        }
        if (liveEstimate != null) {
            Translation2d moved = TagGeometry.reanchor(liveEstimate.toTranslation2d(), before, after);
            liveEstimate = new Translation3d(moved.getX(), moved.getY(), liveEstimate.getZ());
        }
        if (lastFacingYaw.isPresent()) {
            lastFacingYaw = OptionalDouble.of(TagGeometry.reanchorYaw(lastFacingYaw.getAsDouble(), before, after));
        }
        window.clear();
    }

    private static TargetBase reanchor(TargetBase target, Pose2d before, Pose2d after) {
        Translation2d moved = TagGeometry.reanchor(target.position().toTranslation2d(), before, after);
        OptionalDouble facing = target.facingYawRadians().isPresent()
            ? OptionalDouble.of(TagGeometry.reanchorYaw(target.facingYawRadians().getAsDouble(), before, after))
            : OptionalDouble.empty();
        return new TargetBase(new Translation3d(moved.getX(), moved.getY(), target.position().getZ()), facing, target.source());
    }

    public Optional<Sample> getLastSample() {
        return Optional.ofNullable(lastSample);
    }

    public OptionalDouble getFacingYawRadians() {
        return lastFacingYaw;
    }

    public long getAcceptedCount() {
        return acceptedCount;
    }

    public long getRejectedCount() {
        return rejectedCount;
    }

    public ObservationResult getLastRejectReason() {
        return lastRejectReason;
    }

    public int getWindowSize() {
        return window.size();
    }
}
