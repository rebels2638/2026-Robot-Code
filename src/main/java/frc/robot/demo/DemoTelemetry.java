package frc.robot.demo;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/**
 * Publishes a demo value to the dashboard and the log at once. In COMP mode AdvantageKit only writes
 * the log file, so dashboard values have to go to NetworkTables directly.
 */
final class DemoTelemetry {
    private static final String PREFIX = "Demo/";

    private DemoTelemetry() {}

    static void put(String key, double value) {
        SmartDashboard.putNumber(PREFIX + key, value);
        Logger.recordOutput(PREFIX + key, value);
    }

    static void put(String key, boolean value) {
        SmartDashboard.putBoolean(PREFIX + key, value);
        Logger.recordOutput(PREFIX + key, value);
    }

    static void put(String key, String value) {
        SmartDashboard.putString(PREFIX + key, value);
        Logger.recordOutput(PREFIX + key, value);
    }

    static void put(String key, String[] value) {
        SmartDashboard.putStringArray(PREFIX + key, value);
        Logger.recordOutput(PREFIX + key, value);
    }
}
