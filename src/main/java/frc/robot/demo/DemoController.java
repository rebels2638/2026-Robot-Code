package frc.robot.demo;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.Elastic;
import frc.robot.lib.util.ShotCalculator.ShotData;
import frc.robot.lib.util.ShotKinematicsUtil.ShooterKinematics;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.CurrentSystemState;
import frc.robot.subsystems.Superstructure.ShotStatus;
import frc.robot.subsystems.Superstructure.TargetState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

/**
 * Runs the field-trip demo: drive scaling, the two fire modes (manual and AprilTag), saved/placed
 * targets, shot calibration, dashboard telemetry and controller feedback.
 */
public final class DemoController extends SubsystemBase implements DemoShotSource {
    private static final String WELDED_LAYOUT_PATH = "vision/2026-rebuilt-welded.json";
    private static final double SIM_TAG_MAX_RANGE_METERS = 8.0;
    private static final double SIM_TAG_MAX_VIEW_ANGLE_DEG = 75.0;
    private static final double SIM_LATENCY_SECONDS = 0.02;
    private static final DateTimeFormatter SAVED_AT_FORMAT = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");

    private record LerpKey(long heightCentimeters, int correctionVersion, boolean calibration, long hoodTenths, long rpsTenths) {}

    /** Geometry of the current aim point relative to the shooter. */
    private record ShotGeometry(Translation3d aimPoint, double horizontalDistanceMeters, double targetHeightMeters) {}

    private final Superstructure superstructure;
    private final Shooter shooter;
    private final RobotState robotState;
    private final SwerveDrive swerveDrive;
    private final Vision vision;
    private final XboxController controller;
    private final DemoSettings settings;
    private final DemoTagTracker tracker;
    private final CalibrationStore calibrationStore;
    private final AprilTagFieldLayout tagLayout;
    private final Field2d field = new Field2d();
    private final double shooterHeightMeters;
    private final double hoodMinDeg;
    private final double hoodMaxDeg;

    private DemoMode mode = DemoMode.MANUAL;
    private boolean shootHeld = false;
    private List<CalibrationPoint> calibrationPoints = new ArrayList<>();
    private ShotCorrectionTable correctionTable;
    private int correctionVersion = 0;
    private String calibrationSource = "none";
    private String calibrationMessage = "";
    private boolean lastCalibrationEnabled = false;
    private InterpolatingMatrixTreeMap<Double, N3, N1> cachedLerpTable = null;
    private LerpKey cachedLerpKey = null;
    private GentleShotModel.GroundPrediction lastPrediction = null;
    private double lastPredictionTimestamp = Double.NEGATIVE_INFINITY;
    private Translation2d fieldDisplayAnchor = new Translation2d();
    private double lastDetailLogTimestamp = Double.NEGATIVE_INFINITY;
    private int lastAcceptedCamera = -1;
    private int eventCount = 0;

    public DemoController(
        Superstructure superstructure,
        Shooter shooter,
        RobotState robotState,
        SwerveDrive swerveDrive,
        Vision vision,
        XboxController controller
    ) {
        this.superstructure = superstructure;
        this.shooter = shooter;
        this.robotState = robotState;
        this.swerveDrive = swerveDrive;
        this.vision = vision;
        this.controller = controller;
        this.shooterHeightMeters = shooter.getShooterRelativePose().getZ();
        this.hoodMinDeg = shooter.getHoodMinAngleDeg();
        this.hoodMaxDeg = shooter.getHoodMaxAngleDeg();
        this.settings = new DemoSettings(hoodMinDeg, hoodMaxDeg, shooter.getTurretMinAngleDeg(), shooter.getTurretMaxAngleDeg());
        this.tagLayout = loadTagLayout();
        this.tracker = new DemoTagTracker(this::lookupOfficialTagPose, robotState::getPoseAtTime);
        this.calibrationStore = CalibrationStore.forCurrentPlatform();

        vision.setRawObservationListener(this::onRawObservation);
        loadCalibration();
        SmartDashboard.putData("Demo/Field", field);
        registerDashboardButtons();
        logEvent("Demo started in " + mode + " mode");
    }

    // ---------------------------------------------------------------- driver-facing actions

    public double getTranslationScale() {
        return settings.getTranslationScale();
    }

    public double getRotationScale() {
        return settings.getRotationScale();
    }

    public DemoMode getMode() {
        return mode;
    }

    public void setShootHeld(boolean held) {
        shootHeld = held;
        Logger.recordOutput("Demo/Controls/ShootHeld", held);
    }

    public void toggleMode() {
        setMode(mode.toggled());
    }

    public void setMode(DemoMode newMode) {
        mode = newMode;
        rumble(newMode == DemoMode.TAG ? 2 : 1, 0.15);
        logEvent("Mode: " + newMode);
        notify(Elastic.NotificationLevel.INFO, "Fire mode: " + newMode,
            newMode == DemoMode.TAG ? "RT shoots at the AprilTag target" : "RT fires the dashboard manual values");
    }

    /** B button: freeze the live tag as the saved target. */
    public void saveTargetFromTag() {
        double now = Timer.getTimestamp();
        if (tracker.saveLiveTarget(now)) {
            rumble(1, 0.3);
            logEvent("Saved tag target");
            notify(Elastic.NotificationLevel.INFO, "Target saved", "Robot keeps aiming here after the tag is removed");
        } else {
            rumble(3, 0.1);
            logEvent("Save target failed: no tag locked");
            notify(Elastic.NotificationLevel.WARNING, "No tag locked", "Show the tag to a camera, then press B again");
        }
    }

    public void clearSavedTarget() {
        tracker.clearSavedTarget();
        logEvent("Cleared saved target");
        notify(Elastic.NotificationLevel.INFO, "Saved target cleared", "Tag mode follows the live tag again");
    }

    public void placeTarget() {
        Pose2d robotPose = robotState.getEstimatedPose();
        tracker.placeTarget(robotPose, settings.getPlaceForwardMeters(), settings.getPlaceLeftMeters(), settings.getPlaceHeightMeters());
        logEvent(String.format("Placed target %.2f m ahead, %.2f m left, %.2f m high",
            settings.getPlaceForwardMeters(), settings.getPlaceLeftMeters(), settings.getPlaceHeightMeters()));
        notify(Elastic.NotificationLevel.INFO, "Target placed", "Using the hand-placed target");
    }

    /** Sets the current heading to 0 deg and keeps any stored target at the same spot in the room. */
    public void zeroGyro() {
        Pose2d before = robotState.getEstimatedPose();
        robotState.zeroGyro();
        Pose2d after = robotState.getEstimatedPose();
        tracker.onPoseReset(before, after);
        fieldDisplayAnchor = after.getTranslation();
        logEvent("Zeroed gyro");
        notify(Elastic.NotificationLevel.INFO, "Gyro zeroed", "Forward is now the way the robot is facing");
    }

    public void onDisabled() {
        shootHeld = false;
    }

    // ---------------------------------------------------------------- DemoShotSource

    @Override
    public Optional<Translation3d> getTargetLocation() {
        return computeAimPoint(Timer.getTimestamp(), robotState.getEstimatedPose());
    }

    @Override
    public InterpolatingMatrixTreeMap<Double, N3, N1> getLerpTable(double targetHeightMeters) {
        boolean calibration = settings.isCalibrationEnabled();
        double calibrationHood = settings.getCalibrationHoodDeg();
        double calibrationRps = settings.getCalibrationFlywheelRps();
        LerpKey key = new LerpKey(
            Math.round(targetHeightMeters * 100.0),
            correctionVersion,
            calibration,
            calibration ? Math.round(calibrationHood * 10.0) : 0,
            calibration ? Math.round(calibrationRps * 10.0) : 0
        );
        if (cachedLerpTable == null || !key.equals(cachedLerpKey)) {
            cachedLerpTable = buildLerpTable(targetHeightMeters, calibration, calibrationHood, calibrationRps);
            cachedLerpKey = key;
        }
        return cachedLerpTable;
    }

    @Override
    public double getMinShotDistanceMeters() {
        return DemoConstants.MIN_TAG_SHOT_DISTANCE_METERS;
    }

    @Override
    public double getMaxShotDistanceMeters() {
        return DemoConstants.MAX_TAG_SHOT_DISTANCE_METERS;
    }

    @Override
    public double exitSpeedForRps(double flywheelRps) {
        return GentleShotModel.exitSpeedForRps(flywheelRps);
    }

    @Override
    public Optional<String> evaluateShotBlock(ShotData shotData, ShooterKinematics shooterKinematics, Translation3d targetLocation) {
        if (shotData == null) {
            return Optional.of("NO SHOT SOLUTION");
        }
        double heightAboveShooter = targetLocation.getZ() - shooterKinematics.shooterPosition().getZ();
        if (!settings.isCalibrationEnabled()
            && !GentleShotModel.solve(shotData.effectiveDistance(), heightAboveShooter, hoodMinDeg, hoodMaxDeg).feasible()) {
            return Optional.of("UNREACHABLE: target too high for this distance");
        }
        if (shotData.flywheelRPS() > DemoConstants.MAX_MANUAL_FLYWHEEL_RPS) {
            return Optional.of(String.format("TOO FAR: needs %.0f RPS", shotData.flywheelRPS()));
        }

        GentleShotModel.GroundPrediction prediction = GentleShotModel.predict(
            shotData.hoodPitch().getDegrees(),
            shotData.flywheelRPS(),
            shotData.targetFieldYaw().getRadians(),
            shooterKinematics.shooterVxField(),
            shooterKinematics.shooterVyField(),
            heightAboveShooter
        );
        lastPrediction = prediction;
        lastPredictionTimestamp = Timer.getTimestamp();

        double maxSpeed = settings.getMaxSpeedAtTagMps();
        if (maxSpeed > 0.0 && prediction.speedAtTargetMetersPerSec() > maxSpeed) {
            return Optional.of(String.format("TOO FAST AT TAG: %.1f > %.1f m/s, move closer",
                prediction.speedAtTargetMetersPerSec(), maxSpeed));
        }
        double ceiling = settings.getCeilingHeightMeters();
        double peakAboveFloor = shooterKinematics.shooterPosition().getZ() + prediction.peakHeightAboveShooterMeters();
        if (ceiling > 0.0 && peakAboveFloor > ceiling - DemoConstants.CEILING_MARGIN_METERS) {
            return Optional.of(String.format("CEILING: peak %.2f m vs %.2f m ceiling", peakAboveFloor, ceiling));
        }
        return Optional.empty();
    }

    @Override
    public double getManualHoodDegrees() {
        return settings.getManualHoodDeg();
    }

    @Override
    public double getManualTurretDegrees() {
        return settings.getManualTurretDeg();
    }

    @Override
    public double getManualFlywheelRps() {
        return settings.getManualFlywheelRps();
    }

    @Override
    public boolean isManualForceFeed() {
        return settings.isManualForceFeed();
    }

    // ---------------------------------------------------------------- periodic

    @Override
    public void periodic() {
        double now = Timer.getTimestamp();
        Pose2d robotPose = robotState.getEstimatedPose();

        tracker.setTagId(settings.getTagId());
        tracker.update(now);
        if (RobotBase.isSimulation()) {
            injectSimulatedTag(now, robotPose);
        }
        handleCalibrationToggle(now, robotPose);

        if (DriverStation.isTeleopEnabled()) {
            manageSuperstructure(now, robotPose);
            manageDriveMode();
        }
        publishTelemetry(now, robotPose);
    }

    private void manageSuperstructure(double now, Pose2d robotPose) {
        boolean targetAvailable = computeAimPoint(now, robotPose).isPresent();
        if (shootHeld) {
            if (mode == DemoMode.MANUAL) {
                superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.MANUAL_SHOT);
            } else {
                superstructure.setDesiredTargetState(TargetState.DEMO_TAG);
                superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.SHOOTING);
            }
        } else if (mode == DemoMode.TAG && targetAvailable) {
            // Turret follows the target while not firing.
            superstructure.setDesiredTargetState(TargetState.DEMO_TAG);
            superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.TRACKING);
        } else {
            superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        }
    }

    private void manageDriveMode() {
        SwerveDrive.DesiredSystemState current = swerveDrive.getDesiredSystemState();
        boolean teleopDriving = current == SwerveDrive.DesiredSystemState.TELOP_FIELD_RELATIVE
            || current == SwerveDrive.DesiredSystemState.TELOP_ROBOT_RELATIVE;
        if (!teleopDriving) {
            return;
        }
        SwerveDrive.DesiredSystemState wanted = settings.isRobotRelativeDrive()
            ? SwerveDrive.DesiredSystemState.TELOP_ROBOT_RELATIVE
            : SwerveDrive.DesiredSystemState.TELOP_FIELD_RELATIVE;
        if (current != wanted) {
            swerveDrive.setDesiredSystemState(wanted);
        }
    }

    // ---------------------------------------------------------------- targets and shots

    private Optional<Translation3d> computeAimPoint(double now, Pose2d robotPose) {
        Optional<DemoTagTracker.TargetBase> base = tracker.getActiveTarget(now);
        if (base.isEmpty()) {
            return Optional.empty();
        }
        DemoTagTracker.TargetBase target = base.get();
        Translation2d basePosition = target.position().toTranslation2d();
        double facing = target.facingYawRadians().isPresent()
            ? target.facingYawRadians().getAsDouble()
            : TagGeometry.yawTowardRobot(basePosition, robotPose.getTranslation());
        double baseHeight = settings.isHeightOverrideEnabled()
            ? settings.getHeightOverrideMeters()
            : target.position().getZ();
        double aimHeight = MathUtil.clamp(
            baseHeight + settings.getOffsetUpMeters(),
            DemoConstants.MIN_TARGET_HEIGHT_METERS,
            DemoConstants.MAX_TARGET_HEIGHT_METERS
        );
        return Optional.of(TagGeometry.applyOffsets(
            basePosition,
            facing,
            settings.getOffsetOutMeters(),
            settings.getOffsetLeftMeters(),
            aimHeight
        ));
    }

    private Optional<ShotGeometry> computeShotGeometry(double now, Pose2d robotPose) {
        Optional<Translation3d> aimPoint = computeAimPoint(now, robotPose);
        if (aimPoint.isEmpty()) {
            return Optional.empty();
        }
        Translation2d shooterPosition = robotPose.getTranslation().plus(
            shooter.getShooterRelativePose().getTranslation().toTranslation2d().rotateBy(robotPose.getRotation())
        );
        double distance = shooterPosition.getDistance(aimPoint.get().toTranslation2d());
        return Optional.of(new ShotGeometry(aimPoint.get(), distance, aimPoint.get().getZ()));
    }

    private GentleShotModel.Solution modelSolution(double distanceMeters, double targetHeightMeters) {
        return GentleShotModel.solve(distanceMeters, targetHeightMeters - shooterHeightMeters, hoodMinDeg, hoodMaxDeg);
    }

    private InterpolatingMatrixTreeMap<Double, N3, N1> buildLerpTable(
        double targetHeightMeters,
        boolean calibration,
        double calibrationHood,
        double calibrationRps
    ) {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = new InterpolatingMatrixTreeMap<>();
        int rows = 0;
        for (double distance = DemoConstants.MIN_TAG_SHOT_DISTANCE_METERS;
             distance <= DemoConstants.MAX_TAG_SHOT_DISTANCE_METERS + 1e-9;
             distance += DemoConstants.TAG_LERP_TABLE_STEP_METERS) {
            double hood;
            double rps;
            if (calibration) {
                hood = calibrationHood;
                rps = calibrationRps;
            } else {
                GentleShotModel.Solution solution = modelSolution(distance, targetHeightMeters);
                if (!solution.feasible()) {
                    continue;
                }
                ShotCorrectionTable.Correction correction = correctionTable.correctionAt(distance, targetHeightMeters);
                hood = MathUtil.clamp(solution.hoodDegrees() + correction.hoodDegrees(), hoodMinDeg, hoodMaxDeg);
                rps = solution.flywheelRps() + correction.flywheelRps();
            }
            double flightTime = GentleShotModel.flightTimeSeconds(
                distance,
                GentleShotModel.exitSpeedForRps(rps),
                Math.toRadians(hood)
            );
            table.put(distance, row(hood, rps, Double.isFinite(flightTime) ? flightTime : 1.0));
            rows++;
        }
        if (rows == 0) {
            // Nothing reachable at this height. Keep the table non-empty; the shot block reports why.
            table.put(DemoConstants.MIN_TAG_SHOT_DISTANCE_METERS, row(hoodMaxDeg, 0.0, 1.0));
            table.put(DemoConstants.MAX_TAG_SHOT_DISTANCE_METERS, row(hoodMaxDeg, 0.0, 1.0));
        }
        return table;
    }

    private static Matrix<N3, N1> row(double hoodDegrees, double flywheelRps, double flightTimeSeconds) {
        return new Matrix<>(Nat.N3(), Nat.N1(), new double[] {hoodDegrees, flywheelRps, flightTimeSeconds});
    }

    // ---------------------------------------------------------------- vision

    private Optional<Pose3d> lookupOfficialTagPose(int tagId) {
        return tagLayout == null ? Optional.empty() : tagLayout.getTagPose(tagId);
    }

    private static AprilTagFieldLayout loadTagLayout() {
        try {
            return new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve(WELDED_LAYOUT_PATH));
        } catch (IOException deployFailure) {
            try {
                DriverStation.reportWarning("Demo: using WPILib default AprilTag layout (" + deployFailure.getMessage() + ")", false);
                return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            } catch (RuntimeException fallbackFailure) {
                DriverStation.reportError("Demo: no AprilTag layout; tag mode disabled", false);
                return null;
            }
        }
    }

    private void onRawObservation(int cameraIndex, PoseObservation observation) {
        DemoTagTracker.ObservationResult result = tracker.addObservation(cameraIndex, observation);
        if (result == DemoTagTracker.ObservationResult.ACCEPTED) {
            lastAcceptedCamera = cameraIndex;
        }
    }

    private void injectSimulatedTag(double now, Pose2d robotPose) {
        if (!settings.isSimTagEnabled()) {
            return;
        }
        Optional<Pose3d> officialPose = lookupOfficialTagPose(tracker.getTagId());
        if (officialPose.isEmpty()) {
            return;
        }
        double[] simTag = settings.getSimTagPose();
        Pose3d tagInRoom = new Pose3d(simTag[0], simTag[1], simTag[2], new Rotation3d(0.0, 0.0, Math.toRadians(simTag[3])));
        Transform3d robotToTag = new Transform3d(new Pose3d(robotPose), tagInRoom);
        double range = robotToTag.getTranslation().getNorm();
        if (range > SIM_TAG_MAX_RANGE_METERS || !isPrintedSideVisible(tagInRoom, robotPose)) {
            return;
        }
        // What a Limelight would report: the robot pose as if the tag sat at its official pose.
        Pose3d reportedRobotPose = officialPose.get().transformBy(robotToTag.inverse());
        onRawObservation(0, new PoseObservation(
            now - SIM_LATENCY_SECONDS,
            reportedRobotPose,
            0.05,
            1,
            range,
            PoseObservationType.MEGATAG_1,
            tracker.getTagId()
        ));
    }

    private static boolean isPrintedSideVisible(Pose3d tagInRoom, Pose2d robotPose) {
        Translation3d normal = TagGeometry.faceNormal(tagInRoom.getRotation());
        double dx = robotPose.getX() - tagInRoom.getX();
        double dy = robotPose.getY() - tagInRoom.getY();
        double dz = 0.5 - tagInRoom.getZ();
        double length = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (length < 1e-6) {
            return false;
        }
        double cosAngle = (normal.getX() * dx + normal.getY() * dy + normal.getZ() * dz) / length;
        return cosAngle > Math.cos(Math.toRadians(SIM_TAG_MAX_VIEW_ANGLE_DEG));
    }

    // ---------------------------------------------------------------- calibration

    private void loadCalibration() {
        CalibrationStore.LoadResult result = calibrationStore.load();
        calibrationPoints = new ArrayList<>(result.points());
        calibrationSource = result.source();
        if (result.error() != null) {
            calibrationMessage = result.error();
            DriverStation.reportWarning("Demo calibration: " + result.error(), false);
        } else {
            calibrationMessage = "Loaded " + calibrationPoints.size() + " points";
        }
        rebuildCorrectionTable();
        logEvent("Calibration loaded from " + calibrationSource + ": " + calibrationPoints.size() + " points");
    }

    private void rebuildCorrectionTable() {
        correctionTable = new ShotCorrectionTable(calibrationPoints, this::modelSolution);
        correctionVersion++;
        Logger.recordOutput("Demo/Calibration/LoadedPoints", describePoints());
    }

    private void handleCalibrationToggle(double now, Pose2d robotPose) {
        boolean enabled = settings.isCalibrationEnabled();
        if (enabled && !lastCalibrationEnabled) {
            loadCurrentShotIntoCalibration(now, robotPose);
        }
        lastCalibrationEnabled = enabled;
    }

    /** Fills the calibration fields with what the robot would shoot right now. */
    public void loadCurrentShotIntoCalibration() {
        loadCurrentShotIntoCalibration(Timer.getTimestamp(), robotState.getEstimatedPose());
    }

    private void loadCurrentShotIntoCalibration(double now, Pose2d robotPose) {
        Optional<ShotGeometry> geometry = computeShotGeometry(now, robotPose);
        if (geometry.isEmpty()) {
            calibrationMessage = "No target: cannot load current shot";
            return;
        }
        GentleShotModel.Solution solution = modelSolution(geometry.get().horizontalDistanceMeters(), geometry.get().targetHeightMeters());
        if (!solution.feasible()) {
            calibrationMessage = "Target unreachable from here";
            return;
        }
        ShotCorrectionTable.Correction correction = correctionTable.correctionAt(
            geometry.get().horizontalDistanceMeters(),
            geometry.get().targetHeightMeters()
        );
        settings.setCalibrationValues(
            MathUtil.clamp(solution.hoodDegrees() + correction.hoodDegrees(), hoodMinDeg, hoodMaxDeg),
            solution.flywheelRps() + correction.flywheelRps()
        );
        calibrationMessage = String.format("Loaded shot for %.2f m, %.2f m high",
            geometry.get().horizontalDistanceMeters(), geometry.get().targetHeightMeters());
    }

    public void saveCalibrationPoint() {
        if (!settings.isCalibrationEnabled()) {
            calibrationMessage = "Turn on calibration mode first";
            notify(Elastic.NotificationLevel.WARNING, "Calibration is off", calibrationMessage);
            return;
        }
        Optional<ShotGeometry> geometry = computeShotGeometry(Timer.getTimestamp(), robotState.getEstimatedPose());
        if (geometry.isEmpty()) {
            calibrationMessage = "No target: cannot save point";
            notify(Elastic.NotificationLevel.WARNING, "No target", calibrationMessage);
            return;
        }
        CalibrationPoint point = new CalibrationPoint(
            geometry.get().horizontalDistanceMeters(),
            geometry.get().targetHeightMeters(),
            settings.getCalibrationHoodDeg(),
            settings.getCalibrationFlywheelRps(),
            LocalDateTime.now().format(SAVED_AT_FORMAT)
        );
        calibrationPoints.add(point);
        persistCalibration("Saved point: " + point.describe());
    }

    public void deleteLastCalibrationPoint() {
        if (calibrationPoints.isEmpty()) {
            calibrationMessage = "No points to delete";
            return;
        }
        CalibrationPoint removed = calibrationPoints.remove(calibrationPoints.size() - 1);
        persistCalibration("Deleted point: " + removed.describe());
    }

    public void clearCalibrationPoints() {
        calibrationPoints.clear();
        persistCalibration("Cleared all calibration points");
    }

    private void persistCalibration(String message) {
        rebuildCorrectionTable();
        String error = calibrationStore.save(calibrationPoints);
        if (error == null) {
            calibrationSource = calibrationStore.primaryPath().toString();
            calibrationMessage = message;
            rumble(1, 0.2);
            notify(Elastic.NotificationLevel.INFO, "Calibration", message);
        } else {
            calibrationMessage = message + " (NOT SAVED TO FILE: " + error + ")";
            notify(Elastic.NotificationLevel.ERROR, "Calibration file write failed", error);
        }
        logEvent(calibrationMessage);
    }

    private String[] describePoints() {
        String[] lines = new String[calibrationPoints.size()];
        for (int i = 0; i < calibrationPoints.size(); i++) {
            CalibrationPoint point = calibrationPoints.get(i);
            GentleShotModel.Solution model = modelSolution(point.distanceMeters(), point.targetHeightMeters());
            String vsModel = model.feasible()
                ? String.format(" (model %.1f deg, %.1f RPS)", model.hoodDegrees(), model.flywheelRps())
                : " (model: unreachable)";
            lines[i] = (i + 1) + ". " + point.describe() + vsModel;
        }
        return lines;
    }

    // ---------------------------------------------------------------- dashboard

    private void registerDashboardButtons() {
        putButton("Demo/Actions/Zero Gyro", this::zeroGyro);
        putButton("Demo/Actions/Manual Mode", () -> setMode(DemoMode.MANUAL));
        putButton("Demo/Actions/Tag Mode", () -> setMode(DemoMode.TAG));
        putButton("Demo/Target/Save Tag Target", this::saveTargetFromTag);
        putButton("Demo/Target/Clear Saved Target", this::clearSavedTarget);
        putButton("Demo/Target/Place Target", this::placeTarget);
        putButton("Demo/Calibration/Load Current Shot", this::loadCurrentShotIntoCalibration);
        putButton("Demo/Calibration/Save Point", this::saveCalibrationPoint);
        putButton("Demo/Calibration/Delete Last Point", this::deleteLastCalibrationPoint);
        putButton("Demo/Calibration/Clear All Points", this::clearCalibrationPoints);
    }

    private static void putButton(String key, Runnable action) {
        String name = key.substring(key.lastIndexOf('/') + 1);
        SmartDashboard.putData(key, Commands.runOnce(action).ignoringDisable(true).withName(name));
    }

    private void publishTelemetry(double now, Pose2d robotPose) {
        ShotStatus status = superstructure.getShotStatus();
        Optional<ShotGeometry> geometry = computeShotGeometry(now, robotPose);
        Optional<DemoTagTracker.TargetBase> activeTarget = tracker.getActiveTarget(now);
        CurrentSystemState superstructureState = superstructure.getCurrentSystemState();

        DemoTelemetry.put("Status/Mode", mode.name());
        DemoTelemetry.put("Status/Shoot Held", shootHeld);
        DemoTelemetry.put("Status/Ready", isReady(status, superstructureState, geometry.isPresent()));
        DemoTelemetry.put("Status/Reason", describeStatus(status, superstructureState, geometry.isPresent()));
        DemoTelemetry.put("Status/Target Source", activeTarget.map(DemoTagTracker.TargetBase::source).orElse("NONE"));
        DemoTelemetry.put("Status/Superstructure", superstructureState.name());

        publishTagTelemetry(now);
        publishShotTelemetry(now, geometry);
        publishShooterTelemetry();

        DemoTelemetry.put("Drive/Heading Deg", robotPose.getRotation().getDegrees());
        DemoTelemetry.put("Drive/Translation Scale Applied", settings.getTranslationScale());
        DemoTelemetry.put("Drive/Rotation Scale Applied", settings.getRotationScale());

        DemoTelemetry.put("Calibration/Point Count", calibrationPoints.size());
        DemoTelemetry.put("Calibration/Source", calibrationSource);
        DemoTelemetry.put("Calibration/Message", calibrationMessage);
        if (now - lastDetailLogTimestamp >= DemoConstants.TELEMETRY_DETAIL_PERIOD_SEC) {
            lastDetailLogTimestamp = now;
            DemoTelemetry.put("Calibration/Points", describePoints());
        }

        updateField(robotPose, geometry, activeTarget, now);
    }

    private void publishTagTelemetry(double now) {
        DemoTelemetry.put("Tag/Locked", tracker.isLocked(now));
        DemoTelemetry.put("Tag/Seconds Since Seen", Math.min(999.0, tracker.getSecondsSinceSeen(now)));
        DemoTelemetry.put("Tag/Tag ID Used", tracker.getTagId());
        DemoTelemetry.put("Tag/Facing Known", tracker.getFacingYawRadians().isPresent());
        DemoTelemetry.put("Tag/Accepted Frames", tracker.getAcceptedCount());
        DemoTelemetry.put("Tag/Rejected Frames", tracker.getRejectedCount());
        DemoTelemetry.put("Tag/Last Reject", tracker.getLastRejectReason().name());
        DemoTelemetry.put("Tag/Camera", lastAcceptedCamera >= 0 ? vision.getCameraName(lastAcceptedCamera) : "none");
        DemoTelemetry.put("Tag/Saved Target", tracker.getSavedTarget().isPresent());

        Optional<DemoTagTracker.Sample> sample = tracker.getLastSample();
        double distance = Double.NaN;
        double bearing = Double.NaN;
        double height = Double.NaN;
        double tilt = Double.NaN;
        double ambiguity = Double.NaN;
        if (sample.isPresent()) {
            Translation3d relative = sample.get().robotToTag().getTranslation();
            distance = Math.hypot(relative.getX(), relative.getY());
            bearing = Math.toDegrees(Math.atan2(relative.getY(), relative.getX()));
            height = sample.get().position().getZ();
            tilt = sample.get().tiltDegrees();
            ambiguity = sample.get().ambiguity();
            Logger.recordOutput("Demo/Poses/TagRobotRelative", new Pose3d(relative, sample.get().robotToTag().getRotation()));
        }
        DemoTelemetry.put("Tag/Distance M", distance);
        DemoTelemetry.put("Tag/Bearing Deg", bearing);
        DemoTelemetry.put("Tag/Height M", height);
        DemoTelemetry.put("Tag/Tilt Deg", tilt);
        DemoTelemetry.put("Tag/Ambiguity", ambiguity);
    }

    private void publishShotTelemetry(double now, Optional<ShotGeometry> geometry) {
        double distance = Double.NaN;
        double height = Double.NaN;
        double modelHood = Double.NaN;
        double modelRps = Double.NaN;
        double correctionHood = 0.0;
        double correctionRps = 0.0;
        if (geometry.isPresent()) {
            distance = geometry.get().horizontalDistanceMeters();
            height = geometry.get().targetHeightMeters();
            GentleShotModel.Solution solution = modelSolution(distance, height);
            if (solution.feasible()) {
                modelHood = solution.hoodDegrees();
                modelRps = solution.flywheelRps();
            }
            ShotCorrectionTable.Correction correction = correctionTable.correctionAt(distance, height);
            correctionHood = correction.hoodDegrees();
            correctionRps = correction.flywheelRps();
            Logger.recordOutput("Demo/Poses/AimPoint", new Pose3d(geometry.get().aimPoint(), new Rotation3d()));
        }
        DemoTelemetry.put("Shot/Distance M", distance);
        DemoTelemetry.put("Shot/Target Height M", height);
        DemoTelemetry.put("Shot/Model Hood Deg", modelHood);
        DemoTelemetry.put("Shot/Model Flywheel RPS", modelRps);
        DemoTelemetry.put("Shot/Correction Hood Deg", correctionHood);
        DemoTelemetry.put("Shot/Correction Flywheel RPS", correctionRps);

        Optional<ShotData> shotData = superstructure.getCachedShotData();
        boolean tagShot = mode == DemoMode.TAG && geometry.isPresent();
        DemoTelemetry.put("Shot/Hood Deg", tagShot ? shotData.map(data -> data.hoodPitch().getDegrees()).orElse(Double.NaN) : Double.NaN);
        DemoTelemetry.put("Shot/Flywheel RPS", tagShot ? shotData.map(ShotData::flywheelRPS).orElse(Double.NaN) : Double.NaN);
        DemoTelemetry.put("Shot/Flight Time S", tagShot ? shotData.map(ShotData::flightTime).orElse(Double.NaN) : Double.NaN);

        boolean predictionFresh = lastPrediction != null && now - lastPredictionTimestamp < 0.1;
        DemoTelemetry.put("Shot/Speed At Tag MPS", predictionFresh ? lastPrediction.speedAtTargetMetersPerSec() : Double.NaN);
        DemoTelemetry.put("Shot/Peak Height M",
            predictionFresh ? shooterHeightMeters + lastPrediction.peakHeightAboveShooterMeters() : Double.NaN);
        DemoTelemetry.put("Shot/Block Reason", superstructure.getDemoShotBlockReason().orElse(""));
        DemoTelemetry.put("Shot/Calibration Mode", settings.isCalibrationEnabled());
    }

    private void publishShooterTelemetry() {
        DemoTelemetry.put("Shooter/Flywheel RPS", shooter.getFlywheelVelocityRotationsPerSec());
        DemoTelemetry.put("Shooter/Flywheel Setpoint RPS", shooter.getFlywheelSetpointRPS());
        DemoTelemetry.put("Shooter/Hood Deg", shooter.getHoodAngleRotations() * 360.0);
        DemoTelemetry.put("Shooter/Hood Setpoint Deg", shooter.getHoodSetpointRotations() * 360.0);
        DemoTelemetry.put("Shooter/Turret Deg", shooter.getTurretAngleRotations() * 360.0);
        DemoTelemetry.put("Shooter/Turret Setpoint Deg", shooter.getTurretSetpointRotations() * 360.0);
        DemoTelemetry.put("Shooter/Flywheel Ready", shooter.isFlywheelAtSetpoint());
        DemoTelemetry.put("Shooter/Hood Ready", shooter.isHoodAtSetpoint());
        DemoTelemetry.put("Shooter/Turret Ready", shooter.isTurretAtSetpoint());
        DemoTelemetry.put("Shooter/Manual Feed Reason", superstructure.getManualFeedReason().name());
    }

    private boolean isReady(ShotStatus status, CurrentSystemState state, boolean hasTarget) {
        if (mode == DemoMode.MANUAL) {
            return state == CurrentSystemState.MANUAL_SHOOTING;
        }
        return hasTarget && status.readyForShot();
    }

    private String describeStatus(ShotStatus status, CurrentSystemState state, boolean hasTarget) {
        if (mode == DemoMode.MANUAL) {
            if (!shootHeld) {
                return "MANUAL: hold RT to fire";
            }
            return switch (state) {
                case MANUAL_SPINUP -> "SPINNING UP";
                case MANUAL_SHOOTING -> "FIRING (" + superstructure.getManualFeedReason().name() + ")";
                default -> "STARTING";
            };
        }
        return describeTagStatus(
            hasTarget,
            status.distanceInRange(),
            status.effectiveDistanceMeters(),
            superstructure.getDemoShotBlockReason(),
            status.shooterReady(),
            status.hoodAtSetpoint(),
            status.turretAtSetpoint(),
            status.flywheelAtSetpoint(),
            shootHeld,
            settings.isCalibrationEnabled()
        );
    }

    /** Plain-language reason the robot is or is not firing in tag mode. */
    static String describeTagStatus(
        boolean hasTarget,
        boolean distanceInRange,
        double distanceMeters,
        Optional<String> blockReason,
        boolean shooterReady,
        boolean hoodReady,
        boolean turretReady,
        boolean flywheelReady,
        boolean shootHeld,
        boolean calibration
    ) {
        if (!hasTarget) {
            return "NO TARGET: show the tag or place a target";
        }
        if (!distanceInRange) {
            return String.format("OUT OF RANGE: %.1f m (need %.1f to %.1f m)", distanceMeters,
                DemoConstants.MIN_TAG_SHOT_DISTANCE_METERS, DemoConstants.MAX_TAG_SHOT_DISTANCE_METERS);
        }
        if (blockReason.isPresent()) {
            return blockReason.get();
        }
        if (!shooterReady) {
            List<String> waiting = new ArrayList<>();
            if (!hoodReady) {
                waiting.add("hood");
            }
            if (!turretReady) {
                waiting.add("turret");
            }
            if (!flywheelReady) {
                waiting.add("flywheel");
            }
            return "SHOOTER NOT READY: " + String.join(", ", waiting);
        }
        String prefix = calibration ? "CALIBRATING: " : "";
        return prefix + (shootHeld ? "FIRING" : "READY: hold RT");
    }

    private void updateField(
        Pose2d robotPose,
        Optional<ShotGeometry> geometry,
        Optional<DemoTagTracker.TargetBase> activeTarget,
        double now
    ) {
        field.setRobotPose(toDisplay(robotPose.getTranslation(), robotPose.getRotation()));
        Logger.recordOutput("Demo/Poses/Robot", robotPose);

        Optional<DemoTagTracker.TargetBase> live = tracker.getLiveTarget(now);
        if (live.isPresent()) {
            Rotation2d facing = Rotation2d.fromRadians(live.get().facingYawRadians().orElse(0.0));
            field.getObject("Tag").setPose(toDisplay(live.get().position().toTranslation2d(), facing));
            Logger.recordOutput("Demo/Poses/Tag", new Pose3d(live.get().position(), new Rotation3d(0.0, 0.0, facing.getRadians())));
        } else {
            field.getObject("Tag").setPoses();
        }

        Optional<DemoTagTracker.TargetBase> saved = tracker.getSavedTarget();
        if (saved.isPresent()) {
            OptionalDouble facing = saved.get().facingYawRadians();
            field.getObject("Saved").setPose(toDisplay(saved.get().position().toTranslation2d(),
                Rotation2d.fromRadians(facing.orElse(0.0))));
            Logger.recordOutput("Demo/Poses/SavedTarget", new Pose3d(saved.get().position(), new Rotation3d()));
        } else {
            field.getObject("Saved").setPoses();
        }

        if (geometry.isPresent()) {
            field.getObject("Aim").setPose(toDisplay(geometry.get().aimPoint().toTranslation2d(), new Rotation2d()));
        } else {
            field.getObject("Aim").setPoses();
        }
    }

    /** Draws poses relative to the last gyro-zero spot, which sits near the bottom middle of the field view. */
    private Pose2d toDisplay(Translation2d position, Rotation2d rotation) {
        Translation2d relative = position.minus(fieldDisplayAnchor);
        return new Pose2d(
            DemoConstants.FIELD_DISPLAY_ORIGIN_X_METERS + relative.getX(),
            DemoConstants.FIELD_DISPLAY_ORIGIN_Y_METERS + relative.getY(),
            rotation
        );
    }

    // ---------------------------------------------------------------- feedback

    private void logEvent(String message) {
        eventCount++;
        Logger.recordOutput("Demo/Events/Last", message);
        Logger.recordOutput("Demo/Events/Count", eventCount);
        SmartDashboard.putString("Demo/Status/Last Event", message);
    }

    private static void notify(Elastic.NotificationLevel level, String title, String description) {
        Elastic.sendNotification(new Elastic.Notification(level, title, description));
    }

    private void rumble(int pulses, double pulseSeconds) {
        List<Command> steps = new ArrayList<>();
        for (int i = 0; i < pulses; i++) {
            steps.add(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1.0)));
            steps.add(Commands.waitSeconds(pulseSeconds));
            steps.add(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.0)));
            steps.add(Commands.waitSeconds(0.1));
        }
        CommandScheduler.getInstance().schedule(
            Commands.sequence(steps.toArray(Command[]::new)).ignoringDisable(true).withName("DemoRumble")
        );
    }
}
