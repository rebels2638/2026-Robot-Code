package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.DesiredState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.commands.autos.tower.ScoreL1;
// import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    public static RobotContainer instance = null;

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private final XboxController xboxTester;
    private final XboxController xboxDriver;
    private final XboxController xboxOperator;

    private final Shooter shooter = Shooter.getInstance();
    private final RobotState robotState = RobotState.getInstance();
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    // private final Vision vision = Vision.getInstance();

    // Path for follow path state
    private Path currentPath = null;
    private boolean shouldResetPose = false;

    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        // Configure teleop input suppliers for SwerveDrive FSM
        // Using normalized inputs (-1 to 1) with deadband applied
        swerveDrive.setTeleopInputSuppliers(
                () -> -MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
                () -> -MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND));

        // Set up path supplier for SwerveDrive
        swerveDrive.setPathSupplier(() -> currentPath, () -> shouldResetPose);

        // Set default state to TELEOP
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELEOP);

        // Set default superstructure state to HOME
        superstructure.setDesiredState(Superstructure.DesiredState.HOME);

        configureBindings();
    }

    private void configureBindings() {
        // xboxDriver.getAButton().onTrue(
        // new InstantCommand(() ->
        // superstructure.setDesiredState(Superstructure.DesiredState.SHOOTING))
        // ).onFalse(
        // new InstantCommand(() ->
        // superstructure.setDesiredState(Superstructure.DesiredState.READY_FOR_SHOT))
        // );

        // xboxDriver.getXButton().onTrue(
        // new InstantCommand(() ->
        // superstructure.setDesiredState(Superstructure.DesiredState.TRACKING))
        // );

        // xboxDriver.getYButton().onTrue(
        // new InstantCommand(() ->
        // superstructure.setDesiredState(Superstructure.DesiredState.HOME))
        // );

        // xboxDriver.getAButton().onTrue(
        // new InstantCommand(() -> currentPath = new Path(new Waypoint(5,5,new
        // Rotation2d(0)))).andThen(
        // new InstantCommand(() ->
        // swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.FOLLOW_PATH))
        // ).andThen(new WaitUntilCommand(() -> swerveDrive.getCurrentSystemState() ==
        // SwerveDrive.CurrentSystemState.IDLE)).andThen(
        // new InstantCommand(() ->
        // swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELEOP))
        // )
        // );

        xboxDriver.getYButton().onTrue(
                new InstantCommand(() -> robotState.resetPose(new Pose2d(new Translation2d(5, 5), new Rotation2d(0)))));

        xboxDriver.getBButton().whileTrue(
                new ScoreL1());

        Path toClimb = new Path(new Waypoint(1, 3, new Rotation2d(0)));
        xboxDriver.getXButton().onTrue(
                new InstantCommand(() -> currentPath = toClimb).andThen(
                        new InstantCommand(
                                () -> swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.FOLLOW_PATH))
                                .andThen(
                                        new WaitUntilCommand(() -> swerveDrive
                                                .getCurrentSystemState() == SwerveDrive.CurrentSystemState.IDLE))
                                .andThen(
                                        new InstantCommand(() -> swerveDrive
                                                .setDesiredSystemState(SwerveDrive.DesiredSystemState.TELEOP)))));
    }

    public void teleopInit() {
        shouldResetPose = false;

        if (swerveDrive.getCurrentCommand() == null) {
            swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELEOP);
        }

        superstructure.setDesiredState(Superstructure.DesiredState.HOME);
    }

    public void autonomousInit() {
        // Set up for autonomous
        superstructure.setDesiredState(DesiredState.HOME);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.IDLE);
    }

    public void disabledInit() {
        superstructure.setDesiredState(Superstructure.DesiredState.DISABLED);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.DISABLED);
    }

    public Command getAutonomousCommand() {
        currentPath = new Path("lknkn");
        shouldResetPose = true;

        return new InstantCommand(
                () -> swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.PREPARE_FOR_AUTO)).andThen(
                        new WaitUntilCommand(
                                () -> swerveDrive
                                        .getCurrentSystemState() == SwerveDrive.CurrentSystemState.READY_FOR_AUTO))
                .andThen(
                        new InstantCommand(
                                () -> swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.FOLLOW_PATH)));

    }
}
