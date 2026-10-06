package frc.robot.demo;

import frc.robot.constants.FieldConstants;

/** Tunable defaults for the field-trip demo. Dashboard inputs start from these values. */
public final class DemoConstants {
    private DemoConstants() {}

    // Drive stick scaling (fraction of comp max speed). The hard cap keeps the dashboard from
    // ever restoring comp speed by accident.
    public static final double DEFAULT_TRANSLATION_SCALE = 0.25;
    public static final double DEFAULT_ROTATION_SCALE = 0.35;
    public static final double MIN_DRIVE_SCALE = 0.05;
    public static final double MAX_DRIVE_SCALE = 0.5;

    // Manual fire defaults: a gentle 40 deg lob that the comp pass table lands about 3 m out.
    public static final double DEFAULT_MANUAL_FLYWHEEL_RPS = 22.0;
    public static final double DEFAULT_MANUAL_HOOD_DEG = 40.0;
    public static final double DEFAULT_MANUAL_TURRET_DEG = 180.0;
    public static final double MAX_MANUAL_FLYWHEEL_RPS = 60.0;
    public static final double MANUAL_SPINUP_TIMEOUT_SEC = 1.5;
    public static final double MANUAL_TIMEOUT_MIN_SPEED_FRACTION = 0.8;

    // AprilTag target.
    public static final int DEFAULT_TAG_ID = 26;
    public static final double MAX_TAG_RANGE_METERS = FieldConstants.fieldLength;
    /** Window of observations used for the median-smoothed tag position. */
    public static final double TAG_SMOOTHING_WINDOW_SEC = 0.3;
    /** Tag counts as "locked" (fresh) when seen within this time. */
    public static final double TAG_LOCKED_TIMEOUT_SEC = 0.5;
    /** Live target keeps being used (via odometry) this long after the tag was last seen. */
    public static final double LIVE_TARGET_TIMEOUT_SEC = 5.0;
    /** Tag orientation (used only for X/Y offsets) is taken only from frames below this ambiguity. */
    public static final double MAX_ORIENTATION_AMBIGUITY = 0.2;
    /** Below this horizontal component, the tag is too close to flat to have a facing direction. */
    public static final double MIN_FACING_HORIZONTAL_COMPONENT = 0.3;

    // Tag-mode shot range and safety limits.
    public static final double MIN_TAG_SHOT_DISTANCE_METERS = 1.0;
    public static final double MAX_TAG_SHOT_DISTANCE_METERS = 10.0;
    public static final double TAG_LERP_TABLE_STEP_METERS = 0.25;
    public static final double DEFAULT_CEILING_HEIGHT_METERS = 3.0;
    public static final double CEILING_MARGIN_METERS = 0.25;
    public static final double MIN_TARGET_HEIGHT_METERS = 0.0;
    public static final double MAX_TARGET_HEIGHT_METERS = 3.0;

    // Manual "place target" defaults.
    public static final double DEFAULT_PLACE_FORWARD_METERS = 3.0;
    public static final double DEFAULT_PLACE_LEFT_METERS = 0.0;
    public static final double DEFAULT_PLACE_HEIGHT_METERS = 1.2;

    // Calibration storage.
    public static final String CALIBRATION_FILE_NAME = "demo-calibration.json";
    public static final String ROBORIO_CALIBRATION_DIRECTORY = "/home/lvuser";

    // Driver-view field display: where the gyro-zero spot is drawn on the field image.
    public static final double FIELD_DISPLAY_ORIGIN_X_METERS = 1.0;
    public static final double FIELD_DISPLAY_ORIGIN_Y_METERS = FieldConstants.fieldWidth / 2.0;

    public static final double TELEMETRY_DETAIL_PERIOD_SEC = 0.1;
}
