package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.AutoClimbCommand;
import frc.robot.constants.ClimbingConstants;
import frc.robot.constants.ClimbingConstants.AutoClimbTarget;
import frc.robot.constants.Constants;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.JsonUtils;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.DesiredClimbState;
import frc.robot.subsystems.Superstructure.DesiredIntakeState;
import frc.robot.subsystems.Superstructure.DesiredSystemState;
import frc.robot.subsystems.Superstructure.TargetState;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

public final class Autos {
    private static final String PATH_FILE_EXTENSION = ".json";

    private Autos() {}

    public static SendableChooser<Command> getChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do Nothing", Commands.none());

        for (AutoDefinition autoDefinition : autos()) {
            chooser.addOption(autoDefinition.name(), autoDefinition.command());
        }

        return chooser;
    }

    private static List<AutoDefinition> autos() {
        return AutoDefinitionsHolder.AUTOS;
    }

    private static Map<String, Path> preloadPaths() {
        java.nio.file.Path pathDirectory = JsonUtils.PROJECT_ROOT.toPath().resolve("paths");
        Path.DefaultGlobalConstraints defaultGlobalConstraints = JsonUtils.loadGlobalConstraints(JsonUtils.PROJECT_ROOT);
        LinkedHashMap<String, Path> preloadedPaths = new LinkedHashMap<>();

        try (Stream<java.nio.file.Path> pathFiles = Files.list(pathDirectory)) {
            pathFiles
                .filter(Files::isRegularFile)
                .filter(Autos::isPathJsonFile)
                .sorted(Comparator.comparing(pathFile -> pathFile.getFileName().toString()))
                .forEach(pathFile -> {
                    String pathName = getPathName(pathFile);
                    Path previousPath = preloadedPaths.put(
                        pathName,
                        loadPathFromJson(pathFile, defaultGlobalConstraints)
                    );
                    if (previousPath != null) {
                        throw new IllegalStateException("Duplicate auto path name found while preloading: " + pathName);
                    }
                });
        } catch (IOException exception) {
            throw new UncheckedIOException("Failed to preload auto paths from " + pathDirectory, exception);
        }

        return Map.copyOf(preloadedPaths);
    }

    private static boolean isPathJsonFile(java.nio.file.Path pathFile) {
        return pathFile.getFileName().toString().endsWith(PATH_FILE_EXTENSION);
    }

    private static String getPathName(java.nio.file.Path pathFile) {
        String fileName = pathFile.getFileName().toString();
        return fileName.substring(0, fileName.length() - PATH_FILE_EXTENSION.length());
    }

    private static Path loadPathFromJson(
        java.nio.file.Path pathFile,
        Path.DefaultGlobalConstraints defaultGlobalConstraints
    ) {
        try {
            return JsonUtils.loadPathFromJsonString(Files.readString(pathFile), defaultGlobalConstraints);
        } catch (IOException exception) {
            throw new UncheckedIOException("Failed to read auto path JSON: " + pathFile, exception);
        } catch (RuntimeException exception) {
            throw new IllegalStateException("Failed to parse auto path JSON: " + pathFile, exception);
        }
    }

    static Path loadPreloadedPath(String pathName) {
        Path preloadedPath = PreloadedPathsHolder.PRELOADED_PATHS.get(pathName);
        if (preloadedPath == null) {
            throw new IllegalArgumentException("Unknown preloaded auto path: " + pathName);
        }
        return preloadedPath.copy();
    }

    private static final class PreloadedPathsHolder {
        private static final Map<String, Path> PRELOADED_PATHS = preloadPaths();
    }

    private static final class AutoDefinitionsHolder {
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
                "bottom_sweep_over",
                new Pose2d(
                    new Translation2d(3.511, 2.160),
                    Rotation2d.fromRadians(Math.PI)
                ),
                setSystem(DesiredSystemState.HOME),
                firstFollowPath("bottom_sweep_over", false, false)
            ),
            auto(
                "top_sweep_short_depo",
                new Pose2d(
                    new Translation2d(3.83, 5.94),
                    Rotation2d.fromDegrees(165.082)
                ),
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        setSystem(DesiredSystemState.HOME),
                        firstFollowPath("top_sweep_short_depo", false, false)
                    ),
                    new SequentialCommandGroup(
                        new WaitCommand(15.5),
                        setTarget(TargetState.HUB),
                        setSystem(DesiredSystemState.SHOOTING)
                    )
                )

            ),
            auto(
                "straight",
                new Pose2d(
                    new Translation2d(3.511, 2.160),
                    Rotation2d.fromRadians(Math.PI)
                ),
                setSystem(DesiredSystemState.HOME),
                firstFollowPath("straight")
            ),
            auto(
                "outpost",
                new Pose2d(
                    new Translation2d(3.511, 2.160),
                    Rotation2d.fromRadians(Math.PI)
                ),
                setTarget(TargetState.HUB),
                setSystem(DesiredSystemState.SHOOTING),
                firstFollowPath("outpost")
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
                waitSeconds(10)
            ),
            auto(
                "double_swipe_bottom",
                new Pose2d(
                    new Translation2d(3.41, 2.27),
                    Rotation2d.fromDegrees(-147.204)
                ),
                setTarget(TargetState.HUB),
                setSystem(DesiredSystemState.HOME),
                firstFollowPath("top_sweep_short_first", false, true),
                setIntake(DesiredIntakeState.ALTERNATING),
                waitSeconds(7),
                followPath("top_sweep_short_second", false, true),
                setIntake(DesiredIntakeState.ALTERNATING)
            ),
            auto(
                "double_swipe_top",
                new Pose2d(
                    new Translation2d(3.569, 5.68),
                    Rotation2d.fromDegrees(147)
                ),
                setTarget(TargetState.HUB),
                setSystem(DesiredSystemState.HOME),
                firstFollowPath("top_sweep_short_first", false, false),
                setIntake(DesiredIntakeState.ALTERNATING),
                waitSeconds(3.8),
                followPath("top_sweep_short_first", false, false),
                setIntake(DesiredIntakeState.ALTERNATING)
            ),
            auto(
                "double_swipe_top_depo",
                new Pose2d(
                    new Translation2d(3.569, 5.68),
                    Rotation2d.fromDegrees(147)
                ),
                setTarget(TargetState.HUB),
                setSystem(DesiredSystemState.HOME),
                firstFollowPath("top_sweep_short_first", false, false),
                setIntake(DesiredIntakeState.ALTERNATING),
                waitSeconds(2.5),
                followPath(pathWithEndTolerances("top_sweep_hub", 0.5, 70.0), false, false),
                followPath("double_swipe_top_depo", false, false),
                setIntake(DesiredIntakeState.ALTERNATING)
            ),
            auto(
                "top_lob",
                new Pose2d(
                    new Translation2d(3.569, 5.68),
                    Rotation2d.fromDegrees(147)
                ),
                setTarget(TargetState.HUB),
                setSystem(DesiredSystemState.HOME),
                firstFollowPath("top_sweep_lob_first", false, false),
                waitSeconds(1.5),
                followPath("top_sweep_lob_depo", false, false)
            )
        );
    }

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
        return SwerveDrive.getInstance().followPathCommand(
            loadPreloadedPath(pathName),
            shouldResetPose,
            shouldMirrorPath
        );
    }

    private static Command followPath(Path path, boolean shouldResetPose, boolean shouldMirrorPath) {
        return SwerveDrive.getInstance().followPathCommand(path, shouldResetPose, shouldMirrorPath);
    }

    public static Command firstFollowPath(String pathName, boolean shouldResetPose, boolean shouldMirrorPath) {
        Path path = loadPreloadedPath(pathName);
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
            loadPreloadedPath(pathName),
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
        Path path = loadPreloadedPath(pathName);
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

    private static Path pathWithEndTolerances(
        String pathName,
        double endTranslationToleranceMeters,
        double endRotationToleranceDeg
    ) {
        Path path = loadPreloadedPath(pathName);
        path.setPathConstraints(
            path.getPathConstraints()
                .setEndTranslationToleranceMeters(endTranslationToleranceMeters)
                .setEndRotationToleranceDeg(endRotationToleranceDeg)
        );
        return path;
    }

    public static Command waitSeconds(double seconds) {
        return new WaitCommand(seconds);
    }

    private record AutoDefinition(String name, Command command) {}
}
