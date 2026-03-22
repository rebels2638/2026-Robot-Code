package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autos.Autos;
import frc.robot.constants.AlignmentConstants;
// import frc.robot.commands.autos.tower.ScoreL1;
import frc.robot.constants.Constants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.EventTrigger;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.ballistics.ProjectileVisualizer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.TargetState;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;
// import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision;

//: TODO: make sure all lisences in tuner are activated
public class RobotContainer {
    private static final double DRIVE_HELPER_TRIGGER_THRESHOLD = 0.5;

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

    private final RobotState robotState = RobotState.getInstance();
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Vision vision = Vision.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hopper hopper = Hopper.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    private final SendableChooser<Command> autoChooser;

    LoggedNetworkNumber shooterFlywheelSet = new LoggedNetworkNumber("flywheelSet", 0);
    LoggedNetworkNumber turret = new LoggedNetworkNumber("turretSet", 180);
    LoggedNetworkNumber hood = new LoggedNetworkNumber("hoodSet", 70);

    @SuppressWarnings("unused")
    private final ProjectileVisualizer projectileVisualizer = ProjectileVisualizer.getInstance();


    private RobotContainer() {
        registerEventTriggers();

        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        // Configure field-relative teleop input suppliers for SwerveDrive FSM.
        swerveDrive.setFieldRelativeTeleopInputSuppliers(
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND)
        );

        // Configure robot-relative teleop input suppliers for SwerveDrive FSM.
        swerveDrive.setRobotRelativeTeleopInputSuppliers(
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND)
        );
        swerveDrive.setRobotRelativeTeleopHeadingOffset(Rotation2d.fromDegrees(25.0));

        // Default teleop drive mode is field-relative.
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELOP_FIELD_RELATIVE);
        superstructure.setDesiredTargetState(TargetState.HUB);
        
        // Set default superstructure state to HOME
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED);
        superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.RETRACTED);

        autoChooser = Autos.getChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_ALLIANCE_TOP);

        configureBindings();
    }

    private void registerEventTriggers() {
        FollowPath.registerEventTrigger("ready", new InstantCommand(() -> {
            superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.READY_FOR_SHOT);
        }));
        FollowPath.registerEventTrigger("shoot", this::runHubShootingHelper);
        
        FollowPath.registerEventTrigger("intake", () -> {
            superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.INTAKING);
        });
        FollowPath.registerEventTrigger("deploy", () -> {
            superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.DEPLOYED);
        });

        FollowPath.registerEventTrigger("lob", this::runAlliancePassHelper);
    }

    private void configureBindings() {
        bindCompetitionDriveHelpers();
        // xboxDriver.getXButton().onTrue(
        //     new InstantCommand(() -> robotState.resetPose(new Pose2d(robotState.getEstimatedPose().getTranslation(), new Rotation2d(0))))
        // );
        // xboxDriver.getAButton().onTrue(
        //     new InstantCommand(this::runHubShootingHelper)
        // );
        // xboxDriver.getBButton().onTrue(
        //     new InstantCommand(this::runAlliancePassHelper)
        // );
        // xboxDriver.getBButton().onTrue(
        //     new InstantCommand(() -> Shooter.getInstance().setShotVelocity(shooterFlywheelSet.getAsDouble()))
        // );
        // xboxDriver.getXButton().onTrue(
        //     new InstantCommand(() -> Hopper.getInstance().setSetpoint(HopperSetpoint.FEEDING))
        // );
        // xboxDriver.getYButton().onTrue(
        //     new InstantCommand(() -> Shooter.getInstance().setHoodAngle(Rotation2d.fromDegrees(70))).andThen(
        //         new InstantCommand(() -> Shooter.getInstance().setShotVelocity(0))
        //     ).andThen(
        //         new InstantCommand(() -> Hopper.getInstance().setSetpoint(HopperSetpoint.OFF))
        //     )
        // );

        // xboxDriver.getBButton().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.SHOOTING))
        // );
        // xboxDriver.getXButton().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.TRACKING))
        // );
        // );

        // xboxDriver.getAButton().onTrue(
        //     new InstantCommand(() -> shooter.setTurretAngle(new Rotation2d(180
        //     )))
        // );
        // xboxDriver.getBButton().onTrue(
        //     new InstantCommand(() -> shooter.setTurretAngle((Rotation2d.fromDegrees(turret.getAsDouble()))))
        // );

        // xboxDriver.getXButton().onTrue(
        //     new InstantCommand(() -> hopper.setSetpoint(HopperSetpoint.FEEDING))
        // );
        // xboxDriver.getYButton().onTrue(
        //     new InstantCommand(() -> hopper.setSetpoint(HopperSetpoint.OFF))
        // );
                // xboxOperator.getYButton().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.HUB))
        // );
        // xboxOperator.getUpDpad().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_ALLIANCE_TOP))
        // );
        // xboxOperator.getDownDpad().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_ALLIANCE_BOTTOM))
        // );
        // xboxOperator.getLeftBumper().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_NEUTRAL_TOP))
        // );
        // xboxOperator.getRightBumper().onTrue(
        //     new InstantCommand(() -> superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_NEUTRAL_BOTTOM))
        // );

        // Split-state pattern example:
        // superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.TRACKING);
        // superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.INTAKING);

        // xboxDriver.getAButton().onTrue(
        //     new InstantCommand(() -> Intake.getInstance().setSetpoint(Intake.IntakeSetpoint.INTAKING))
        // );
        // xboxDriver.getBButton().onTrue(
        //     new InstantCommand(() -> Intake.getInstance().setSetpoint(Intake.IntakeSetpoint.STOWED))
        // );

        // Test snap-to-angle bindings with split superstructure states:
        // xboxDriver.getAButton().onTrue(new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.BUMP)));
        // xboxDriver.getAButton().onFalse(new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME)));
    }

    public Command buildIntakeBumpCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.INTAKING)),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> intake.getPivotAngleRotations()/360 > 10),
                new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED))
            ),
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.INTAKING))
        );
    }
    private void bindCompetitionDriveHelpers() {
        Trigger leftBumperTrigger = xboxDriver.getLeftBumper();
        Trigger rightBumperTrigger = xboxDriver.getRightBumper();

        leftBumperTrigger.onTrue(buildIntakeBumpCommand());

        rightBumperTrigger.onTrue(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED))
        );

        xboxDriver.getLeftTriggerButton(DRIVE_HELPER_TRIGGER_THRESHOLD).onTrue(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.INTAKING))
        ).onFalse(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.DEPLOYED))
        );

        xboxDriver.getRightTriggerButton(DRIVE_HELPER_TRIGGER_THRESHOLD).onTrue(
            new InstantCommand(this::runHubShootingHelper)
        ).onFalse(
            new InstantCommand(this::runHubTrackingHelper)
        );

        xboxDriver.getYButton().onTrue(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.REVERSING))
        ).onFalse(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.DEPLOYED))
        );

        xboxDriver.getXButton().onTrue(
            new InstantCommand(() -> swerveDrive.setDesiredSystemState(
                SwerveDrive.DesiredSystemState.TELOP_ROBOT_RELATIVE
            ))
        ).onFalse(
            new InstantCommand(() -> swerveDrive.setDesiredSystemState(
                SwerveDrive.DesiredSystemState.TELOP_FIELD_RELATIVE
            ))
        );

        xboxDriver.getBButton().whileTrue(
            new RunCommand(this::runAlliancePassHelper)
        ).onFalse(
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.TRACKING))
        );
    }

    private void runHubShootingHelper() {
        superstructure.setDesiredTargetState(TargetState.HUB);
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.SHOOTING);
    }

    private void runHubTrackingHelper() {
        superstructure.setDesiredTargetState(TargetState.HUB);
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.TRACKING);
    }

    private void runAlliancePassHelper() {
        superstructure.getClosestLineOfSightAlliancePassTarget().ifPresentOrElse(
            targetState -> {
                superstructure.setDesiredTargetState(targetState);
                superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.SHOOTING);
            },
            () -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME)
        );
    }

    public void teleopInit() {
        // Ensure we're in teleop state
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELOP_FIELD_RELATIVE);
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED);
        // dont set climb state leave up to driver
    }

    public void autonomousInit() {
        // Set up for autonomous
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.RETRACTED);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.IDLE);
    }

    public void disabledInit() {
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.DISABLED);
        superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.DISABLED);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.DISABLED);
    }

    public Command getAutonomousCommand() {
        Command selectedAuto = autoChooser.getSelected();
        return selectedAuto != null ? selectedAuto : Commands.none();
        // return Autos.followPath("bottom_sweep_long", true, true);
    }
}
