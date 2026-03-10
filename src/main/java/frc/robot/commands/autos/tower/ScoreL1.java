package frc.robot.commands.autos.tower;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.AlignmentConstants;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.lib.util.IntermediateGenerator;
import frc.robot.lib.util.RebelUtil;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveDrive.CurrentSystemState;
import frc.robot.subsystems.swerve.SwerveDrive.DesiredSystemState;

public class ScoreL1 extends Command {
    private final SwerveDrive swerveDrive;
    private final RobotState robotState;
    private final IntermediateGenerator.GeneratorMode intermediateMode;
    private boolean isFinished = false;

    public ScoreL1() {
        this(IntermediateGenerator.GeneratorMode.DEFAULT);
    }

    public ScoreL1(IntermediateGenerator.GeneratorMode intermediateMode) {
        this.swerveDrive = SwerveDrive.getInstance();
        this.robotState = RobotState.getInstance();
        this.intermediateMode = intermediateMode;
    }

    @Override
    public void initialize() {
        isFinished = false;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        Pose2d currentPose = robotState.getEstimatedPose();

        double blueZoneX = AlignmentConstants.AllianceBounds.blueZoneLineX;
        double redZoneX = AlignmentConstants.AllianceBounds.redZoneLineX;

        boolean behindBoundLine = isRed ? (currentPose.getX() > redZoneX) : (currentPose.getX() < blueZoneX);

        if (!behindBoundLine) {
            isFinished = true;
            return;
        }

        Logger.recordOutput("ScoreL1/Status", "Running");

        Pose2d currentPoseBlue = RebelUtil.applyFlip(currentPose);
        Pose2d towerPoseBlue = Constants.ScoreTowerConstants.TOWER_WAYPOINTS[0];
        double minDistanceToTower = towerPoseBlue.getTranslation().getDistance(currentPoseBlue.getTranslation());
        int towerPoseIndex = 0;

        for (int i = 0; i < Constants.ScoreTowerConstants.TOWER_WAYPOINTS.length; i++) {
            Pose2d pose = Constants.ScoreTowerConstants.TOWER_WAYPOINTS[i];
            double distance = pose.getTranslation().getDistance(currentPoseBlue.getTranslation());

            if (distance < minDistanceToTower) {
                minDistanceToTower = distance;

                towerPoseBlue = pose;
                towerPoseIndex = i;
            }
        }

        List<Path.PathElement> pathElements = new ArrayList<>();
        pathElements.add(new Waypoint(currentPoseBlue));

        boolean needsIntermediate = IntermediateGenerator.shouldUseIntermediate(
            intermediateMode,
            currentPoseBlue,
            towerPoseBlue);
        Translation2d[] intermediateTranslation = IntermediateGenerator.intermediateTranslations(
            intermediateMode,
            currentPoseBlue,
            towerPoseIndex);

        if (needsIntermediate) {
            for (int i = intermediateTranslation.length - 1; i >= 0; i--) {
                pathElements.add(new TranslationTarget(intermediateTranslation[i]));
            }
        }

        pathElements.add(new Waypoint(towerPoseBlue, 0.08));

        int finalTranslationOrdinal = pathElements.size() - 2;
        PathConstraints constraints = new PathConstraints()
                .setMaxVelocityMetersPerSec(
                    new Path.RangedConstraint(
                        Constants.ScoreTowerConstants.APPROACH_MAX_VELOCITY_METERS_PER_SEC,
                        0,
                        finalTranslationOrdinal),
                    new Path.RangedConstraint(
                        Constants.ScoreTowerConstants.MAX_VELOCITY_METERS_PER_SEC,
                        finalTranslationOrdinal,
                        pathElements.size() - 1))
                .setMaxAccelerationMetersPerSec2(Constants.ScoreTowerConstants.MAX_ACCELERATION_METERS_PER_SEC2);

        Path toClimb = new Path(pathElements, constraints);

        swerveDrive.setPathSupplier(() -> toClimb, () -> false);
        swerveDrive.setDesiredSystemState(DesiredSystemState.FOLLOW_PATH);
    }

    @Override
    public void execute() {
        Logger.recordOutput("ScoreL1/Execute/SwerveState", swerveDrive.getCurrentSystemState().toString());
        if (swerveDrive.getCurrentSystemState() == CurrentSystemState.IDLE) {
            isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("ScoreL1/End/Interrupted", interrupted);
        swerveDrive.setDesiredSystemState(DesiredSystemState.TELEOP);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
