// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Mode currentMode = Mode.SIM; // TODO: change this if sim
    public static boolean agentMode = false;

    // public static final boolean isSYSID = true; // TODO: change this if sysid

    public static enum Mode {
        /** Running on a real robot. */
        COMP,

        DEV,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double kLOOP_CYCLE_MS;
    static {
        switch (currentMode) {
            case COMP:
                kLOOP_CYCLE_MS = 0.02;

                break;

            case DEV:
                kLOOP_CYCLE_MS = 0.02;

                break;

            case SIM:
                kLOOP_CYCLE_MS = 0.02;

                break;

            case REPLAY:
                kLOOP_CYCLE_MS = 0.02;

                break;

            default:
                kLOOP_CYCLE_MS = 0.02;

                break;
        }
    }

    public static final class OperatorConstants {
        public static final int kDRIVER_CONTROLLER_PORT = 0;

        // Joystick Deadband
        // yes, this high
        public static final double LEFT_X_DEADBAND = 0.09;
        public static final double LEFT_Y_DEADBAND = 0.09;

        public static final double RIGHT_X_DEADBAND = 0.12;

        private OperatorConstants() {
        }
    }

    public static final class VisionConstants {

        private VisionConstants() {}
    }

    public static final class AlignmentConstants {

        private AlignmentConstants() {}
    }

    public static final class FieldConstants {
        public static final Translation3d kSHOOTER_TARGET = new Translation3d(49.85, 50.0, 0.46);
        private FieldConstants() {}
    }

    public static boolean shouldFlipPath() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
