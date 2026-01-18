package frc.robot.commands.autos.tower;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.constants.MechAElementConstants;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveDrive.CurrentSystemState;
import frc.robot.subsystems.swerve.SwerveDrive.DesiredSystemState;

public class ScoreL1 extends Command {
    private final SwerveDrive swerveDrive;
    private final RobotState robotState;
    private boolean isFinished = false;

    public ScoreL1() {
        this.swerveDrive = SwerveDrive.getInstance();
        this.robotState = RobotState.getInstance();
    }

    @Override
    public void initialize() {
        isFinished = false;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        Pose2d currentPose = robotState.getEstimatedPose();

        double blueZoneX = MechAElementConstants.AllianceBounds.blueZoneLineX;
        double redZoneX = MechAElementConstants.AllianceBounds.redZoneLineX;
        Pose2d hub = AllianceFlipUtil.applyFlip(new Pose2d(MechAElementConstants.Hub.hubCenter, new Rotation2d()), isRed);

        boolean behindBoundLine = isRed ? (currentPose.getX() > redZoneX) : (currentPose.getX() < blueZoneX);
        boolean yInBounds = Math.abs(currentPose.getY() - hub.getY()) <= 2.0;
        boolean xInBounds = Math.abs(currentPose.getX() - hub.getX()) <= 2.5;

        if (!behindBoundLine || !yInBounds || !xInBounds) {
            isFinished = true;
            return;
        }

        Logger.recordOutput("ScoreL1/Status", "Running");

        // bline bug?
        Pose2d currentPoseBlue = isRed ? AllianceFlipUtil.applyFlip(currentPose, true) : currentPose;
        Pose2d towerPose = MechAElementConstants.Tower.towerPoses[0];
        double minDistance = currentPoseBlue.getTranslation().getDistance(towerPose.getTranslation());

        for (int i = 1; i < MechAElementConstants.Tower.towerPoses.length; i++) {
            Pose2d candidate = MechAElementConstants.Tower.towerPoses[i];
            double distance = currentPoseBlue.getTranslation().getDistance(candidate.getTranslation());

            if (distance < minDistance) {
                minDistance = distance;
                towerPose = candidate;
            }
        }

        Path toClimb = new Path(
                new PathConstraints().setMaxVelocityMetersPerSec(2.8).setMaxAccelerationMetersPerSec2(8.0),
                new Waypoint(currentPoseBlue, 0.5),
                new Waypoint(towerPose, 0.2));

        swerveDrive.setPathSupplier(() -> toClimb, () -> false);
        swerveDrive.setDesiredSystemState(DesiredSystemState.FOLLOW_PATH);
    }

    @Override
    public void execute() {
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
