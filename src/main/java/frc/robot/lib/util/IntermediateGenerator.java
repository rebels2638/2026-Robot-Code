package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.AlignmentConstants.Tower;
import frc.robot.constants.Constants;

public final class IntermediateGenerator {
    public enum GeneratorMode {
        DEFAULT
    }

    private IntermediateGenerator() {}

    public static boolean shouldUseIntermediate(
            GeneratorMode mode,
            Pose2d currentPoseBlue,
            Pose2d towerPoseBlue) {
        return strategyFor(mode).shouldUseIntermediate(currentPoseBlue, towerPoseBlue);
    }

    public static Translation2d[] intermediateTranslations(
            GeneratorMode mode,
            Pose2d currentPoseBlue,
            int towerPoseIndex) {
        return strategyFor(mode).intermediateTranslations(currentPoseBlue, towerPoseIndex);
    }

    private static Strategy strategyFor(GeneratorMode mode) {
        switch (mode) {
            case DEFAULT:
            default:
                return DefaultStrategy.INSTANCE;
        }
    }

    private interface Strategy {
        boolean shouldUseIntermediate(Pose2d currentPoseBlue, Pose2d towerPoseBlue);

        Translation2d[] intermediateTranslations(Pose2d currentPoseBlue, int towerPoseIndex);
    }

    private static final class DefaultStrategy implements Strategy {
        private static final DefaultStrategy INSTANCE = new DefaultStrategy();

        @Override
        public boolean shouldUseIntermediate(Pose2d currentPoseBlue, Pose2d towerPoseBlue) {
            return isBehindTower(currentPoseBlue, towerPoseBlue) || isInsideTower(currentPoseBlue);
        }

        @Override
        public Translation2d[] intermediateTranslations(Pose2d currentPoseBlue, int towerPoseIndex) {
            if (shouldUseIntermediate(currentPoseBlue, Constants.ScoreTowerConstants.TOWER_WAYPOINTS[towerPoseIndex])) {
                // left
                if (towerPoseIndex == 0) {
                    return new Translation2d[] {
                        new Translation2d(
                            Constants.ScoreTowerConstants.TOWER_WAYPOINTS[towerPoseIndex].getX()
                                + Constants.ScoreTowerConstants.INTERMEDIATE_CLEARANCE_METERS.getX(),
                            Tower.towerPoses[towerPoseIndex].getY()
                                - Constants.ScoreTowerConstants.INTERMEDIATE_CLEARANCE_METERS.getY()),
                        isInsideTower(currentPoseBlue)
                            ? new Translation2d(
                                Constants.RobotConstants.robotWidth / 2.0 + 0.1,
                                Tower.towerPoses[towerPoseIndex].getY()
                                    - Constants.ScoreTowerConstants.INTERMEDIATE_CLEARANCE_METERS.getY())
                            : currentPoseBlue.getTranslation()
                    };
                    // middle and right
                } else {
                    return new Translation2d[] {
                        new Translation2d(
                            Tower.towerPoses[2].getX()
                                + Constants.ScoreTowerConstants.INTERMEDIATE_CLEARANCE_METERS.getX(),
                            Tower.towerPoses[towerPoseIndex].getY()
                                + Constants.ScoreTowerConstants.INTERMEDIATE_CLEARANCE_METERS.getY()),
                        isInsideTower(currentPoseBlue)
                            ? new Translation2d(
                                Constants.RobotConstants.robotWidth / 2.0 + 0.1,
                                Tower.towerPoses[towerPoseIndex].getY()
                                    + Constants.ScoreTowerConstants.INTERMEDIATE_CLEARANCE_METERS.getY())
                            : currentPoseBlue.getTranslation()
                    };
                }
            }

            return new Translation2d[] {};
        }

        private boolean isInsideTower(Pose2d currentPoseBlue) {
            double robotRadius = Constants.RobotConstants.robotWidth / 2.0;

            return currentPoseBlue.getX() < Tower.towerPoses[0].getX()
                    && currentPoseBlue.getY()
                            > (Tower.towerPoses[1].getY() - Tower.towerWidth / 2.0 - robotRadius)
                    && currentPoseBlue.getY()
                            < (Tower.towerPoses[1].getY() + Tower.towerWidth / 2.0 + robotRadius);
        }

        private boolean isBehindTower(Pose2d currentPoseBlue, Pose2d towerPoseBlue) {
            return (currentPoseBlue.getX() < towerPoseBlue.getX() + Constants.RobotConstants.robotWidth / 2.0);
        }
    }
}
