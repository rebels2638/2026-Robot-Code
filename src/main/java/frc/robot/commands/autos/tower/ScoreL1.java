package frc.robot.commands.autos.tower;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.constants.MechAElementConstants;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.lib.util.AllianceFlippingUtil;
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
        double hubY = MechAElementConstants.Hub.hubCenter.getY();

        boolean behindBoundLine = isRed ? (currentPose.getX() > redZoneX) : (currentPose.getX() < blueZoneX);
        boolean yInBounds = Math.abs(currentPose.getY() - hubY) <= 1.5;
        boolean xInBounds = Math.abs(
                currentPose.getX() - AllianceFlippingUtil.applyFlip(MechAElementConstants.Hub.hubCenter, isRed).getX()
            ) <= 2.0;

        if (!behindBoundLine || !yInBounds || !xInBounds) {
            isFinished = true;
            return;
        }

        Logger.recordOutput("ScoreL1/Status", "Running");

        Pose2d towerPoseBlue = MechAElementConstants.Tower.towerPoses[0];
        Pose2d currentPoseBlue = AllianceFlippingUtil.applyFlip(currentPose, true);
        double minDistanceToTower = towerPoseBlue.getTranslation().getDistance(currentPoseBlue.getTranslation());

        for (Pose2d pose : MechAElementConstants.Tower.towerPoses) {
            double distance = pose.getTranslation().getDistance(currentPoseBlue.getTranslation());

            if (distance < minDistanceToTower) {
                minDistanceToTower = distance;
                towerPoseBlue = pose;
            }
        }

        Path toClimb = new Path(
                new PathConstraints().setMaxVelocityMetersPerSec(2.65).setMaxAccelerationMetersPerSec2(8.0),
                new Waypoint(currentPoseBlue),
                new Waypoint(towerPoseBlue));

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
