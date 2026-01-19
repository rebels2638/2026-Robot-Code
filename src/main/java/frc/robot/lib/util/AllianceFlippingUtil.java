package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.MechAElementConstants;

public class AllianceFlippingUtil {
    public static Rotation2d applyFlip(Rotation2d rotation, boolean isRed) {
        if (isRed) {
            return rotation.plus(new Rotation2d(Math.PI));
        } else {
            return rotation;
        }
    }

    public static Translation2d applyFlip(Translation2d translation, boolean isRed) {
        if (isRed) {
            double fieldLength = MechAElementConstants.fieldLength;
            double fieldWidth = MechAElementConstants.fieldWidth;

            double flippedX = fieldLength - translation.getX();
            double flippedY = fieldWidth - translation.getY();

            return new Translation2d(flippedX, flippedY);
        } else {
            return translation;
        }
    }

    public static Pose2d applyFlip(Pose2d pose, boolean isRed) {
        Translation2d translation = pose.getTranslation();
        Rotation2d rotation = pose.getRotation();

        if (isRed) {
            return new Pose2d(applyFlip(translation, true), applyFlip(rotation, true));
        } else {
            return pose;
        }
    }
}
