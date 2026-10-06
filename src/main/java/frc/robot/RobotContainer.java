package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autos.Autos;
import frc.robot.constants.Constants;
import frc.robot.demo.DemoController;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.ballistics.ProjectileVisualizer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.TargetState;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.FlywheelSetpoint;
import frc.robot.subsystems.shooter.Shooter.HoodSetpoint;
import frc.robot.subsystems.shooter.Shooter.TurretSetpoint;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

//: TODO: make sure all lisences in tuner are activated
public class RobotContainer {
    private static final double DRIVE_HELPER_TRIGGER_THRESHOLD = 0.5;

    // Flip this single flag when you want dashboard-driven shooter testing instead of
    // the normal competition container wiring.
    private static final boolean USE_MANUAL_SHOOTER_TEST_CONFIG = false;

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
    private final Superstructure superstructure;
    /** Field-trip demo controls; null unless Constants.DEMO_MODE. */
    private final DemoController demoController;
    private final SendableChooser<Command> autoChooser;

    private final LoggedNetworkNumber manualFlywheelRps = new LoggedNetworkNumber("flywheelSet", 0.0);
    private final LoggedNetworkNumber manualTurretDegrees = new LoggedNetworkNumber("turretSet", 180.0);
    private final LoggedNetworkNumber manualHoodDegrees = new LoggedNetworkNumber("hoodSet", 70.0);

    @SuppressWarnings("unused")
    private final ProjectileVisualizer projectileVisualizer = ProjectileVisualizer.getInstance();

    private RobotContainer() {
        superstructure = USE_MANUAL_SHOOTER_TEST_CONFIG ? null : Superstructure.getInstance();

        registerEventTriggers();

        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        demoController = Constants.DEMO_MODE && superstructure != null
            ? new DemoController(superstructure, shooter, robotState, swerveDrive, vision, xboxDriver)
            : null;
        if (demoController != null) {
            superstructure.setDemoShotSource(demoController);
        }

        configureDriveInputSuppliers();
        configureInitialState();

        autoChooser = Autos.getChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private boolean isManualShooterTestConfig() {
        return superstructure == null;
    }

    private void configureDriveInputSuppliers() {
        swerveDrive.setFieldRelativeTeleopInputSuppliers(
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND) * getTranslationScale(),
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND) * getTranslationScale(),
            () -> -MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND) * getRotationScale()
        );

        swerveDrive.setRobotRelativeTeleopInputSuppliers(
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND) * getTranslationScale(),
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND) * getTranslationScale(),
            () -> -MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND) * getRotationScale()
        );
        swerveDrive.setRobotRelativeTeleopHeadingOffset(Rotation2d.fromDegrees(25.0));
    }

    /** Demo slows the sticks; comp is full speed. */
    private double getTranslationScale() {
        return demoController != null ? demoController.getTranslationScale() : 1.0;
    }

    private double getRotationScale() {
        return demoController != null ? demoController.getRotationScale() : 1.0;
    }

    private void configureInitialState() {
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELOP_FIELD_RELATIVE);

        if (isManualShooterTestConfig()) {
            stopManualShooterTest();
            hopper.setSetpoint(HopperSetpoint.OFF);
            return;
        }

        superstructure.setDesiredTargetState(TargetState.HUB);
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED);
        superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.RETRACTED);
    }

    private void registerEventTriggers() {
        if (isManualShooterTestConfig()) {
            return;
        }

        FollowPath.registerEventTrigger(
            "ready",
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.READY_FOR_SHOT))
        );
        FollowPath.registerEventTrigger("shoot", this::runHubShootingHelper);

        FollowPath.registerEventTrigger(
            "intake",
            () -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.INTAKING)
        );
        FollowPath.registerEventTrigger(
            "deploy",
            () -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.DEPLOYED)
        );
        FollowPath.registerEventTrigger(
            "alt",
            () -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.ALTERNATING)
        );
        FollowPath.registerEventTrigger("lob", this::runAlliancePassHelper);
        FollowPath.registerEventTrigger("lob_top_corner", this::runAllianceTopCornerPassHelper);
        FollowPath.registerEventTrigger("lob_bottom_corner", this::runAllianceBottomCornerPassHelper);
        FollowPath.registerEventTrigger(
            "reverse",
            () -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.REVERSING)
        );
    }

    private void configureBindings() {
        if (isManualShooterTestConfig()) {
            configureManualShooterTestBindings();
            return;
        }

        configureCompetitionBindings();
    }

    private void configureCompetitionBindings() {
        if (demoController != null) {
            bindDemoControls();
            return;
        }
        bindCompetitionDriveHelpers();
    }

    /**
     * Single-controller demo layout: intake/hopper buttons as at comp, RT fires in the current mode,
     * A switches Manual/Tag, B saves the tag target. Back, Start and the D-pad are unused.
     */
    private void bindDemoControls() {
        bindIntakeAndHopperHelpers();

        xboxDriver.getRightTriggerButton(DRIVE_HELPER_TRIGGER_THRESHOLD).onTrue(
            new InstantCommand(() -> demoController.setShootHeld(true))
        ).onFalse(
            new InstantCommand(() -> demoController.setShootHeld(false))
        );

        xboxDriver.getAButton().onTrue(Commands.runOnce(demoController::toggleMode).ignoringDisable(true));
        xboxDriver.getBButton().onTrue(Commands.runOnce(demoController::saveTargetFromTag).ignoringDisable(true));
    }

    private void configureManualShooterTestBindings() {
        shooter.setHoodAngleSupplier(() -> Rotation2d.fromDegrees(manualHoodDegrees.getAsDouble()));
        shooter.setFlywheelRPSSupplier(() -> manualFlywheelRps.getAsDouble());

        xboxDriver.getAButton().onTrue(buildApplyManualShooterTestCommand());
        xboxDriver.getBButton().onTrue(new InstantCommand(() -> hopper.setSetpoint(HopperSetpoint.FEEDING)));
        xboxDriver.getYButton().onTrue(new InstantCommand(() -> hopper.setSetpoint(HopperSetpoint.OFF)));
        xboxDriver.getXButton().onTrue(buildStopManualShooterTestCommand());
    }

    private Command buildApplyManualShooterTestCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> shooter.setHoodSetpoint(HoodSetpoint.DYNAMIC)),
            Commands.runOnce(() -> shooter.setTurretAngle(Rotation2d.fromDegrees(manualTurretDegrees.getAsDouble()))),
            Commands.runOnce(() -> shooter.setFlywheelSetpoint(FlywheelSetpoint.DYNAMIC))
        );
    }

    private Command buildStopManualShooterTestCommand() {
        return Commands.runOnce(this::stopManualShooterTest);
    }

    private void stopManualShooterTest() {
        shooter.setFlywheelSetpoint(FlywheelSetpoint.OFF);
        shooter.setHoodSetpoint(HoodSetpoint.HOME);
        shooter.setTurretSetpoint(TurretSetpoint.HOME);
    }

    private Command buildIntakeBumpCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.INTAKING)),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> intake.getPivotAngleRotations() / 360 > 10),
                new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED))
            ),
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.INTAKING))
        );
    }

    private void bindCompetitionDriveHelpers() {
        bindIntakeAndHopperHelpers();

        xboxDriver.getRightTriggerButton(DRIVE_HELPER_TRIGGER_THRESHOLD).onTrue(
            new InstantCommand(this::runHubShootingHelper)
        ).onFalse(
            new InstantCommand(this::runHubTrackingHelper)
        );

        xboxDriver.getAButton().whileTrue(
            Commands.startEnd(
                () -> robotState.setZeroGyroButtonHeld(true),
                () -> robotState.setZeroGyroButtonHeld(false)
            ).ignoringDisable(true)
        );

        xboxDriver.getBButton().whileTrue(
            new RunCommand(this::runAlliancePassHelper)
        ).onFalse(
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.TRACKING))
        );
    }

    /** Intake and hopper buttons shared by the comp and demo layouts. */
    private void bindIntakeAndHopperHelpers() {
        Trigger leftBumperTrigger = xboxDriver.getLeftBumper();
        Trigger rightBumperTrigger = xboxDriver.getRightBumper();

        // leftBumperTrigger.onTrue(buildIntakeBumpCommand());

        leftBumperTrigger.onTrue(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.ALTERNATING))
        ).onFalse(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.DEPLOYED))
        );

        rightBumperTrigger.onTrue(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED))
        );

        xboxDriver.getLeftTriggerButton(DRIVE_HELPER_TRIGGER_THRESHOLD).onTrue(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.INTAKING))
        ).onFalse(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.DEPLOYED))
        );

        xboxDriver.getYButton().onTrue(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.REVERSING))
        ).onFalse(
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.DEPLOYED))
        );

        xboxDriver.getXButton().onTrue(
            new InstantCommand(() -> superstructure.setDesiredHopperState(Superstructure.DesiredHopperState.REVERSE))
        ).onFalse(
            new InstantCommand(() -> superstructure.setDesiredHopperState(Superstructure.DesiredHopperState.DEFAULT))
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

    private void runAllianceTopCornerPassHelper() {
        runExplicitAlliancePassHelper(TargetState.PASS_ALLIANCE_TOP_CORNER);
    }

    private void runAllianceBottomCornerPassHelper() {
        runExplicitAlliancePassHelper(TargetState.PASS_ALLIANCE_BOTTOM_CORNER);
    }

    private void runExplicitAlliancePassHelper(TargetState targetState) {
        superstructure.setDesiredTargetState(targetState);
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.SHOOTING);
    }

    public void teleopInit() {
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELOP_FIELD_RELATIVE);

        if (isManualShooterTestConfig()) {
            return;
        }

        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.DEPLOYED);
        superstructure.setDesiredHopperState(Superstructure.DesiredHopperState.DEFAULT);
        // dont set climb state leave up to driver
    }

    public void autonomousInit() {
        if (isManualShooterTestConfig()) {
            stopManualShooterTest();
            hopper.setSetpoint(HopperSetpoint.OFF);
            swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.IDLE);
            return;
        }

        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.RETRACTED);
        superstructure.setDesiredHopperState(Superstructure.DesiredHopperState.DEFAULT);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.IDLE);
    }

    public void disabledInit() {
        if (isManualShooterTestConfig()) {
            stopManualShooterTest();
            hopper.setSetpoint(HopperSetpoint.OFF);
            swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.DISABLED);
            return;
        }

        if (demoController != null) {
            demoController.onDisabled();
        }
        superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.DISABLED);
        superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.DISABLED);
        superstructure.setDesiredHopperState(Superstructure.DesiredHopperState.DEFAULT);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.DISABLED);
    }

    public Command getAutonomousCommand() {
        if (demoController != null) {
            // Demo: autonomous does nothing, so an accidental Auto enable can never run a path.
            return Commands.none();
        }
        Command selectedAuto = autoChooser.getSelected();
        return selectedAuto != null ? selectedAuto : Commands.none();
    }
}
