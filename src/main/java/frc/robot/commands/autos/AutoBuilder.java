package frc.robot.commands.autos;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveDrive.CurrentSystemState;
import frc.robot.subsystems.swerve.SwerveDrive.DesiredSystemState;
import frc.robot.commands.autos.shooting.AutoShootingController;
import frc.robot.commands.autos.shooting.AutoShootingZoneManager;

/**
 * Simple utility class for creating autonomous routines.
 *
 * Usage:
 * AutoBuilder.path("my_path") // Single path
 * AutoBuilder.paths("path1", "path2") // Multiple sequential paths
 * AutoBuilder.sequence(path1Cmd, intakeCmd, path2Cmd) // Custom sequence
 */
public class AutoBuilder {

    private static final SwerveDrive swerveDrive = SwerveDrive.getInstance();

    public static Command path(String pathName) {
        return path(pathName, true);
    }

    public static Command path(String pathName, boolean resetPose) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    Logger.recordOutput("Auto/CurrentPath", pathName);
                    Path path = new Path(pathName);
                    swerveDrive.setPathSupplier(() -> path, () -> resetPose);
                    swerveDrive.setDesiredSystemState(DesiredSystemState.PREPARE_FOR_AUTO);
                }),

                new WaitUntilCommand(() -> swerveDrive.getCurrentSystemState() == CurrentSystemState.READY_FOR_AUTO),
                Commands.runOnce(() -> swerveDrive.setDesiredSystemState(DesiredSystemState.FOLLOW_PATH)),

                new WaitUntilCommand(() -> swerveDrive.getCurrentSystemState() == CurrentSystemState.IDLE),
                Commands.runOnce(() -> Logger.recordOutput("Auto/PathComplete", pathName)));
    }

    public static Command paths(String... pathNames) {
        SequentialCommandGroup seq = new SequentialCommandGroup();
        for (int i = 0; i < pathNames.length; i++) {
            seq.addCommands(path(pathNames[i], i == 0));
        }
        return seq;
    }

    public static Command sequence(Command... commands) {
        return Commands.sequence(commands);
    }
    public static Command withShooting(Command autoCommand) {
        AutoShootingZoneManager.getInstance().reset();
        return Commands.parallel(
                new AutoShootingController(),
                autoCommand);
    }

    public static Command wait(double seconds) {
        return Commands.waitSeconds(seconds);
    }
}
