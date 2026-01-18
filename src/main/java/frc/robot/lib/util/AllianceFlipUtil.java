package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.MechAElementConstants;

public class AllianceFlipUtil {
    public static Pose2d applyFlip(Pose2d pose, boolean isRed) {
        Translation2d flippedTranslation =
            new Translation2d(
                MechAElementConstants.fieldLength - pose.getX(),
                MechAElementConstants.fieldWidth - pose.getY());

        Rotation2d flippedRotation = new Rotation2d(Math.PI).minus(pose.getRotation());

        return isRed ? new Pose2d(flippedTranslation, flippedRotation) : pose;
    }
}
