package frc.robot.commands.autos;

 import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.AutoClimbCommand;
import frc.robot.constants.ClimbingConstants;
import frc.robot.constants.ClimbingConstants.AutoClimbTarget;
import frc.robot.constants.Constants;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.DesiredClimbState;
import frc.robot.subsystems.Superstructure.DesiredIntakeState;
import frc.robot.subsystems.Superstructure.DesiredSystemState;
import frc.robot.subsystems.Superstructure.TargetState;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.ArrayList;
import java.util.List;

public final class Autos {
    private Autos() {}

    public static SendableChooser<Command> getChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do Nothing", Commands.none());

        for (AutoDefinition autoDefinition : AUTOS) {
            chooser.addOption(autoDefinition.name(), autoDefinition.command());
        }

        return chooser;
    }

    private static final List<AutoDefinition> AUTOS = List.of(
        // auto(
        //     "Simple Back",
        //     new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()),
        //     setSystem(DesiredSystemState.HOME),
        //     setIntake(DesiredIntakeState.STOWED),
        //     setClimber(DesiredClimbState.RETRACTED),
        //     followPath("simple_back", true)
        // ),
        // auto(
        //     "Citrus Sweep",
        //     new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()),
        //     setSystem(DesiredSystemState.HOME),
        //     setIntake(DesiredIntakeState.DEPLOYED),
        //     setClimber(DesiredClimbState.RETRACTED),
        //     followPath("citrus_sweep"),
        //     waitSeconds(0.25),
        //     setIntake(DesiredIntakeState.STOWED)
        // )
        auto(
            "outpost",
            new Pose2d(
                new Translation2d(3.511, 2.160),
                Rotation2d.fromRadians(Math.PI)
            ),
            setTarget(TargetState.HUB),
            setSystem(DesiredSystemState.SHOOTING),
            firstFollowPath("outpost"),
            waitSeconds(10),
            climb()
        ),
        auto(
            "overcharge",
            new Pose2d(
                new Translation2d(3.569, 2.320),
                Rotation2d.fromRadians(0)
            ),
            setTarget(TargetState.HUB),
            setSystem(DesiredSystemState.HOME),
            firstFollowPath("bottom_jab_sharp"),
            waitSeconds(5),
            followPath("outpost"),
            waitSeconds(10),
            climb()
        ),
        auto(
            "double_swipe",
            new Pose2d(
                new Translation2d(3.569, 2.320),
                Rotation2d.fromRadians(0)
            ),
            setTarget(TargetState.HUB),
            setSystem(DesiredSystemState.HOME),
            firstFollowPath("bottom_jab_sharp"),
            waitSeconds(6),
            followPath("bottom_sweep_short"),
            waitSeconds(6)
        )

    );

    private static AutoDefinition auto(String name, Command... commands) {
        SequentialCommandGroup auto = new SequentialCommandGroup(commands);
        auto.setName(name);
        return new AutoDefinition(name, auto);
    }

    private static AutoDefinition auto(String name, Pose2d simResetPose, Command... commands) {
        ArrayList<Command> autoCommands = new ArrayList<>();
        if (shouldInsertSimResetPose(simResetPose, Constants.currentMode)) {
            autoCommands.add(resetPoseForSim(simResetPose));
        }
        autoCommands.addAll(List.of(commands));
        return auto(name, autoCommands.toArray(Command[]::new));
    }

    public static Command setIntake(DesiredIntakeState intakeState) {
        return new InstantCommand(() -> Superstructure.getInstance().setDesiredIntakeState(intakeState));
    }

    public static Command setSystem(DesiredSystemState systemState) {
        return new InstantCommand(() -> Superstructure.getInstance().setDesiredSystemState(systemState));
    }

    public static Command setClimber(DesiredClimbState climbState) {
        return new InstantCommand(() -> Superstructure.getInstance().setDesiredClimbState(climbState));
    }

    public static Command setTarget(TargetState targetState) {
        return new InstantCommand(() -> Superstructure.getInstance().setDesiredTargetState(targetState));
    }

    public static Command climb() {
        return new AutoClimbCommand(ClimbingConstants.TOWER);
    }

    public static Command followPath(String pathName) {
        return followPath(pathName, false, false);
    }

    public static Command firstFollowPath(String pathName) {
        return firstFollowPath(pathName, false, false);
    }

    public static Command followPath(String pathName, boolean shouldResetPose) {
        return followPath(pathName, shouldResetPose, false);
    }

    public static Command firstFollowPath(String pathName, boolean shouldResetPose) {
        return firstFollowPath(pathName, shouldResetPose, false);
    }

    public static Command followPath(String pathName, boolean shouldResetPose, boolean shouldMirrorPath) {
        return SwerveDrive.getInstance().followPathCommand(new Path(pathName), shouldResetPose, shouldMirrorPath);
    }

    public static Command firstFollowPath(String pathName, boolean shouldResetPose, boolean shouldMirrorPath) {
        Path path = new Path(pathName);
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        return swerveDrive.prepareForPathCommand(path, shouldMirrorPath).andThen(
            swerveDrive.followPathCommand(path, shouldResetPose, shouldMirrorPath)
        );
    }

    public static Command followPath(
        String pathName,
        boolean shouldResetPose,
        boolean shouldMirrorPath,
        boolean shouldFlipPath
    ) {
        return SwerveDrive.getInstance().followPathCommand(
            new Path(pathName),
            shouldResetPose,
            shouldMirrorPath,
            shouldFlipPath
        );
    }

    public static Command firstFollowPath(
        String pathName,
        boolean shouldResetPose,
        boolean shouldMirrorPath,
        boolean shouldFlipPath
    ) {
        Path path = new Path(pathName);
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        return swerveDrive.prepareForPathCommand(path, shouldMirrorPath, shouldFlipPath).andThen(
            swerveDrive.followPathCommand(
                path,
                shouldResetPose,
                shouldMirrorPath,
                shouldFlipPath
            )
        );
    }

    static boolean shouldInsertSimResetPose(Pose2d simResetPose, Constants.Mode currentMode) {
        return simResetPose != null && currentMode == Constants.Mode.SIM;
    }

    static Pose2d resolveSimResetPose(Pose2d simResetPose, boolean shouldFlipPath) {
        if (simResetPose == null) {
            throw new IllegalArgumentException("simResetPose cannot be null when resolving the sim reset pose");
        }
        return shouldFlipPath ? FlippingUtil.flipFieldPose(simResetPose) : simResetPose;
    }

    private static Command resetPoseForSim(Pose2d simResetPose) {
        return new InstantCommand(() ->
            RobotState.getInstance().resetPose(resolveSimResetPose(simResetPose, Constants.shouldFlipPath()))
        );
    }

    public static Command waitSeconds(double seconds) {
        return new WaitCommand(seconds);
    }

    private record AutoDefinition(String name, Command command) {}
}
