// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.configs.SwerveConfig;
import frc.robot.constants.FieldConstants.Tower;
import frc.robot.lib.util.ConfigLoader;

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

        /** Running a physics simulaton
         * r. */
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
        // yes, this high :sob:
        public static final double LEFT_X_DEADBAND = 0.09;
        public static final double LEFT_Y_DEADBAND = 0.09;

        public static final double RIGHT_X_DEADBAND = 0.12;

        private OperatorConstants() {}
    }

    // public static final class AlignmentConstants {
    //     public static final double MAX_VELOCITY_METERS_PER_SEC = 2.65;
        
    //     public static final double APPROACH_MAX_VELOCITY_METERS_PER_SEC = 2.15;
    //     public static final Translation2d INTERMEDIATE_CLEARANCE_METERS = new Translation2d(
    //             RobotConstants.robotRadius / 2.0 - 0.3, RobotConstants.robotRadius / 2.0 + 0.35);
    //     public static final double DUPLICATE_TRANSLATION_EPSILON_METERS = 1e-3;

    //     public static final Pose2d[] TOWER_WAYPOINTS = {
    //             new Pose2d(
    //                     Units.inchesToMeters(59.76),
    //                     Units.inchesToMeters(141.2),
    //                     Rotation2d.fromDegrees(180.0)),
    //             new Pose2d(
    //                     Units.inchesToMeters(59.76),
    //                     Units.inchesToMeters(149.094),
    //                     Rotation2d.fromDegrees(180.0)),
    //             new Pose2d(
    //                     Units.inchesToMeters(59.76),
    //                     Units.inchesToMeters(163.0),
    //                     Rotation2d.fromDegrees(180.0))
    //     };

    //     public static boolean shouldUseIntermediate(Pose2d currentPoseBlue, Pose2d towerPoseBlue) {
    //         return isBehindTower(currentPoseBlue, towerPoseBlue) || isInsideTower(currentPoseBlue);
    //     }

    //     public static Translation2d[] intermediateTranslation(Pose2d currentPoseBlue, int towerPoseIndex) {
    //         if (shouldUseIntermediate(currentPoseBlue, TOWER_WAYPOINTS[towerPoseIndex])) {
    //             // left
    //             if (towerPoseIndex == 0) {
    //                 return new Translation2d[] {
    //                     new Translation2d(
    //                         TOWER_WAYPOINTS[towerPoseIndex].getX() + INTERMEDIATE_CLEARANCE_METERS.getX(),
    //                         Tower.towerPoses[towerPoseIndex].getY() - INTERMEDIATE_CLEARANCE_METERS.getY()),
    //                     isInsideTower(currentPoseBlue) ? new Translation2d(
    //                         RobotConstants.robotRadius / 2.0 + 0.1,
    //                         Tower.towerPoses[towerPoseIndex].getY() - INTERMEDIATE_CLEARANCE_METERS.getY()) : currentPoseBlue.getTranslation()
    //                 };
    //                 // middle and right
    //             } else {
    //                 return new Translation2d[] {
    //                     new Translation2d(
    //                         Tower.towerPoses[2].getX() + INTERMEDIATE_CLEARANCE_METERS.getX(),
    //                         Tower.towerPoses[towerPoseIndex].getY() + INTERMEDIATE_CLEARANCE_METERS.getY()),
    //                     isInsideTower(currentPoseBlue) ? new Translation2d(
    //                         RobotConstants.robotRadius / 2.0 + 0.1,
    //                         Tower.towerPoses[towerPoseIndex].getY() + INTERMEDIATE_CLEARANCE_METERS.getY()) : currentPoseBlue.getTranslation()
    //                 };
    //             }
    //         }

    //         return new Translation2d[]{};
    //     }

    //     public static boolean isInsideTower(Pose2d currentPoseBlue) {
    //         double robotRadius = RobotConstants.robotRadius / 2.0;

    //         return currentPoseBlue.getX() < Tower.towerPoses[0].getX() &&
    //                 currentPoseBlue.getY() > (Tower.towerPoses[1].getY() - Tower.towerWidth / 2.0 - robotRadius) &&
    //                 currentPoseBlue.getY() < (Tower.towerPoses[1].getY() + Tower.towerWidth / 2.0 + robotRadius);
    //     }

    //     public static boolean isBehindTower(Pose2d currentPoseBlue, Pose2d towerPoseBlue) {
    //         return (currentPoseBlue.getX() < towerPoseBlue.getX() +
    //                 RobotConstants.robotRadius / 2.0);
    //     }

    //     private AlignmentConstants() {}
    // }

    // public static final class RobotConstants {
    //     static SwerveConfig swerveConfig = ConfigLoader.load("swerve", SwerveConfig.class);

    //     public static final double robotRadius = Math.hypot(
    //             swerveConfig.drivetrain.frontLeftX - swerveConfig.drivetrain.backRightX,
    //             swerveConfig.drivetrain.frontLeftY - swerveConfig.drivetrain.backRightY
    //     ) / 2.0;

    //     private RobotConstants() {}
    // }

    public static boolean shouldFlipPath() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }

        return false;
    }
}
