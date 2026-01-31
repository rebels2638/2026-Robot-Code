package frc.robot.commands.autos;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.constants.AlignmentConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.CurrentState;
import frc.robot.subsystems.Superstructure.DesiredState;

public class AutoShootingController extends Command {
    private final Superstructure superstructure;
    private final RobotState robotState;
    private final AutoShootingZoneManager zoneManager;

    public AutoShootingController() {
        this.superstructure = Superstructure.getInstance();
        this.robotState = RobotState.getInstance();
        this.zoneManager = AutoShootingZoneManager.getInstance();
    }

    @Override
    public void initialize() {
        Logger.recordOutput("AutoShootingController/active", true);
    }

    @Override
    public void execute() {
        boolean inShootingZone = zoneManager.isInShootingZone();
        boolean inAllianceZone = isInAllianceZone();
        boolean hasTarget = robotState.hasRecentVisionTarget();

        Logger.recordOutput("AutoShootingController/inShootingZone", inShootingZone);
        Logger.recordOutput("AutoShootingController/inAllianceZone", inAllianceZone);
        Logger.recordOutput("AutoShootingController/hasTarget", hasTarget);

        if (inShootingZone && inAllianceZone && hasTarget) {
            CurrentState currentState = superstructure.getCurrentState();

            if (currentState == CurrentState.READY_FOR_SHOT || currentState == CurrentState.SHOOTING) {
                superstructure.setDesiredState(DesiredState.SHOOTING);
                Logger.recordOutput("AutoShootingController/action", "SHOOTING");
            } else {
                superstructure.setDesiredState(DesiredState.READY_FOR_SHOT);
                Logger.recordOutput("AutoShootingController/action", "PREPARING");
            }
        } else if (inShootingZone && hasTarget) {
            superstructure.setDesiredState(DesiredState.TRACKING);
            Logger.recordOutput("AutoShootingController/action", "TRACKING");
        } else if (inShootingZone) {
            superstructure.setDesiredState(DesiredState.HOME);
            Logger.recordOutput("AutoShootingController/action", "NO_TARGET");
        } else {
            superstructure.setDesiredState(DesiredState.HOME);
            Logger.recordOutput("AutoShootingController/action", "HOME");
        }
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.setDesiredState(DesiredState.HOME);
        Logger.recordOutput("AutoShootingController/active", false);
        Logger.recordOutput("AutoShootingController/action", "ENDED");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Checks if the robot is currently within its alliance zone.
     * Uses the alliance zone boundary from AlignmentConstants.
     *
     * @return true if robot is behind the alliance zone line
     */
    private boolean isInAllianceZone() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose2d currentPose = robotState.getEstimatedPose();

        double blueZoneX = AlignmentConstants.AllianceBounds.blueZoneLineX;
        double redZoneX = AlignmentConstants.AllianceBounds.redZoneLineX;

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return currentPose.getX() > redZoneX;
        } else {
            return currentPose.getX() < blueZoneX;
        }
    }
}
