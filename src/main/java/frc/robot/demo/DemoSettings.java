package frc.robot.demo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Dashboard inputs for the demo, all under {@code /SmartDashboard/Demo/}. Every value has a safe
 * default and is clamped, so a missing dashboard or a bad entry never produces an unsafe value.
 * AdvantageKit records every input in the log.
 */
public final class DemoSettings {
    static final String ROOT = "/SmartDashboard/Demo/";

    private final double hoodMinDeg;
    private final double hoodMaxDeg;
    private final double turretMinDeg;
    private final double turretMaxDeg;

    private final LoggedNetworkNumber translationScale =
        number("Drive/Translation Scale", DemoConstants.DEFAULT_TRANSLATION_SCALE);
    private final LoggedNetworkNumber rotationScale =
        number("Drive/Rotation Scale", DemoConstants.DEFAULT_ROTATION_SCALE);
    private final LoggedNetworkBoolean robotRelativeDrive = bool("Drive/Robot Relative", false);

    private final LoggedNetworkNumber manualFlywheelRps =
        number("Manual/Flywheel RPS", DemoConstants.DEFAULT_MANUAL_FLYWHEEL_RPS);
    private final LoggedNetworkNumber manualHoodDeg = number("Manual/Hood Deg", DemoConstants.DEFAULT_MANUAL_HOOD_DEG);
    private final LoggedNetworkNumber manualTurretDeg =
        number("Manual/Turret Deg", DemoConstants.DEFAULT_MANUAL_TURRET_DEG);
    private final LoggedNetworkBoolean manualForceFeed = bool("Manual/Force Feed", false);

    private final LoggedNetworkNumber tagId = number("Tag/Tag ID", DemoConstants.DEFAULT_TAG_ID);
    private final LoggedNetworkNumber offsetOut = number("Tag/Offset Out M", 0.0);
    private final LoggedNetworkNumber offsetLeft = number("Tag/Offset Left M", 0.0);
    private final LoggedNetworkNumber offsetUp = number("Tag/Offset Up M", 0.0);
    private final LoggedNetworkBoolean heightOverrideEnabled = bool("Tag/Height Override", false);
    private final LoggedNetworkNumber heightOverrideMeters =
        number("Tag/Override Height M", DemoConstants.DEFAULT_PLACE_HEIGHT_METERS);
    private final LoggedNetworkNumber maxSpeedAtTag =
        number("Tag/Max Speed At Tag MPS", DemoConstants.DEFAULT_MAX_SPEED_AT_TAG_MPS);
    private final LoggedNetworkNumber ceilingHeight =
        number("Tag/Ceiling Height M", DemoConstants.DEFAULT_CEILING_HEIGHT_METERS);

    private final LoggedNetworkNumber placeForward =
        number("Target/Place Forward M", DemoConstants.DEFAULT_PLACE_FORWARD_METERS);
    private final LoggedNetworkNumber placeLeft = number("Target/Place Left M", DemoConstants.DEFAULT_PLACE_LEFT_METERS);
    private final LoggedNetworkNumber placeHeight =
        number("Target/Place Height M", DemoConstants.DEFAULT_PLACE_HEIGHT_METERS);

    private final LoggedNetworkBoolean calibrationEnabled = bool("Calibration/Enabled", false);
    private final LoggedNetworkNumber calibrationHoodDeg =
        number("Calibration/Hood Deg", DemoConstants.DEFAULT_MANUAL_HOOD_DEG);
    private final LoggedNetworkNumber calibrationFlywheelRps =
        number("Calibration/Flywheel RPS", DemoConstants.DEFAULT_MANUAL_FLYWHEEL_RPS);

    // Simulation-only fake tag, so the whole tag pipeline can be exercised on a laptop.
    private final LoggedNetworkBoolean simTagEnabled;
    private final LoggedNetworkNumber simTagX;
    private final LoggedNetworkNumber simTagY;
    private final LoggedNetworkNumber simTagZ;
    private final LoggedNetworkNumber simTagFacingDeg;

    public DemoSettings(double hoodMinDeg, double hoodMaxDeg, double turretMinDeg, double turretMaxDeg) {
        this.hoodMinDeg = hoodMinDeg;
        this.hoodMaxDeg = hoodMaxDeg;
        this.turretMinDeg = turretMinDeg;
        this.turretMaxDeg = turretMaxDeg;
        if (RobotBase.isSimulation()) {
            simTagEnabled = bool("Sim/Tag Enabled", true);
            simTagX = number("Sim/Tag X M", 4.0);
            simTagY = number("Sim/Tag Y M", 0.0);
            simTagZ = number("Sim/Tag Z M", 1.2);
            simTagFacingDeg = number("Sim/Tag Facing Deg", 180.0);
        } else {
            simTagEnabled = null;
            simTagX = null;
            simTagY = null;
            simTagZ = null;
            simTagFacingDeg = null;
        }
    }

    private static LoggedNetworkNumber number(String key, double defaultValue) {
        return new LoggedNetworkNumber(ROOT + key, defaultValue);
    }

    private static LoggedNetworkBoolean bool(String key, boolean defaultValue) {
        return new LoggedNetworkBoolean(ROOT + key, defaultValue);
    }

    /** Clamps to [min, max]; a non-finite value falls back to the default. */
    static double clampOr(double value, double min, double max, double fallback) {
        if (!Double.isFinite(value)) {
            return fallback;
        }
        return MathUtil.clamp(value, min, max);
    }

    public double getTranslationScale() {
        return clampOr(translationScale.get(), DemoConstants.MIN_DRIVE_SCALE, DemoConstants.MAX_DRIVE_SCALE,
            DemoConstants.DEFAULT_TRANSLATION_SCALE);
    }

    public double getRotationScale() {
        return clampOr(rotationScale.get(), DemoConstants.MIN_DRIVE_SCALE, DemoConstants.MAX_DRIVE_SCALE,
            DemoConstants.DEFAULT_ROTATION_SCALE);
    }

    public boolean isRobotRelativeDrive() {
        return robotRelativeDrive.get();
    }

    public double getManualFlywheelRps() {
        return clampOr(manualFlywheelRps.get(), 0.0, DemoConstants.MAX_MANUAL_FLYWHEEL_RPS,
            DemoConstants.DEFAULT_MANUAL_FLYWHEEL_RPS);
    }

    public double getManualHoodDeg() {
        return clampOr(manualHoodDeg.get(), hoodMinDeg, hoodMaxDeg, DemoConstants.DEFAULT_MANUAL_HOOD_DEG);
    }

    public double getManualTurretDeg() {
        return clampOr(manualTurretDeg.get(), turretMinDeg, turretMaxDeg, DemoConstants.DEFAULT_MANUAL_TURRET_DEG);
    }

    public boolean isManualForceFeed() {
        return manualForceFeed.get();
    }

    public int getTagId() {
        double value = tagId.get();
        if (!Double.isFinite(value) || value < 0.0 || value > 1000.0) {
            return DemoConstants.DEFAULT_TAG_ID;
        }
        return (int) Math.round(value);
    }

    public double getOffsetOutMeters() {
        return clampOr(offsetOut.get(), -5.0, 5.0, 0.0);
    }

    public double getOffsetLeftMeters() {
        return clampOr(offsetLeft.get(), -5.0, 5.0, 0.0);
    }

    public double getOffsetUpMeters() {
        return clampOr(offsetUp.get(), -3.0, 3.0, 0.0);
    }

    public boolean isHeightOverrideEnabled() {
        return heightOverrideEnabled.get();
    }

    public double getHeightOverrideMeters() {
        return clampOr(heightOverrideMeters.get(), DemoConstants.MIN_TARGET_HEIGHT_METERS,
            DemoConstants.MAX_TARGET_HEIGHT_METERS, DemoConstants.DEFAULT_PLACE_HEIGHT_METERS);
    }

    /** Zero or less turns the limit off. */
    public double getMaxSpeedAtTagMps() {
        return clampOr(maxSpeedAtTag.get(), 0.0, 30.0, DemoConstants.DEFAULT_MAX_SPEED_AT_TAG_MPS);
    }

    /** Zero or less turns the limit off. */
    public double getCeilingHeightMeters() {
        return clampOr(ceilingHeight.get(), 0.0, 20.0, DemoConstants.DEFAULT_CEILING_HEIGHT_METERS);
    }

    public double getPlaceForwardMeters() {
        return clampOr(placeForward.get(), 0.5, 15.0, DemoConstants.DEFAULT_PLACE_FORWARD_METERS);
    }

    public double getPlaceLeftMeters() {
        return clampOr(placeLeft.get(), -10.0, 10.0, DemoConstants.DEFAULT_PLACE_LEFT_METERS);
    }

    public double getPlaceHeightMeters() {
        return clampOr(placeHeight.get(), DemoConstants.MIN_TARGET_HEIGHT_METERS,
            DemoConstants.MAX_TARGET_HEIGHT_METERS, DemoConstants.DEFAULT_PLACE_HEIGHT_METERS);
    }

    public boolean isCalibrationEnabled() {
        return calibrationEnabled.get();
    }

    public double getCalibrationHoodDeg() {
        return clampOr(calibrationHoodDeg.get(), hoodMinDeg, hoodMaxDeg, DemoConstants.DEFAULT_MANUAL_HOOD_DEG);
    }

    public double getCalibrationFlywheelRps() {
        return clampOr(calibrationFlywheelRps.get(), 0.0, DemoConstants.MAX_MANUAL_FLYWHEEL_RPS,
            DemoConstants.DEFAULT_MANUAL_FLYWHEEL_RPS);
    }

    /** Pre-fills the calibration fields (shows on the dashboard next loop). */
    public void setCalibrationValues(double hoodDeg, double flywheelRps) {
        calibrationHoodDeg.set(Math.round(hoodDeg * 10.0) / 10.0);
        calibrationFlywheelRps.set(Math.round(flywheelRps * 10.0) / 10.0);
    }

    public boolean isSimTagEnabled() {
        return simTagEnabled != null && simTagEnabled.get();
    }

    public double[] getSimTagPose() {
        if (simTagX == null) {
            return new double[] {4.0, 0.0, 1.2, 180.0};
        }
        return new double[] {
            clampOr(simTagX.get(), -50.0, 50.0, 4.0),
            clampOr(simTagY.get(), -50.0, 50.0, 0.0),
            clampOr(simTagZ.get(), 0.0, 3.0, 1.2),
            clampOr(simTagFacingDeg.get(), -360.0, 360.0, 180.0)
        };
    }
}
