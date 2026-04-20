package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand; 
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.CurrentSystemState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Dashboard {
    private static Field2d robotField;
    private static SwerveDrive swerveDrive;

    private static StringPublisher superstructureCurrentState;
    private static StringPublisher superStructureDesiredState;
    private static StringPublisher swerveDriveCommand;
    private static StringPublisher swerveDriveCurrentState;
    private static StringPublisher swerveDriveDesiredState;
    private static DoublePublisher matchTimePublisher;
    private static BooleanPublisher flywheelAtSetpoint;
    private static BooleanPublisher hoodAtSetpoint;
    private static BooleanPublisher turretAtSetpoint;
    private static BooleanPublisher shotValid;
    
    private static SendableChooser<Command> autoChooser;
    private static SendableBuilderImpl autoBuilder;

    // FIX: Promoted from local variables to static fields so update() can be called each loop
    private static SendableBuilderImpl fieldBuilder;
    private static SendableBuilderImpl swerveBuilder;
    
    private static boolean initialized = false;

    public static void addAutoChooserOption(String key, Command command) {
        autoChooser.addOption(key, command);
    }

    public static Command getCurrentAutoCommand() { 
        return autoChooser.getSelected();
    }

    private static void initialize() {
        if (initialized) return;
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("Dashboard");

        // Create publishers once
        superstructureCurrentState = table.getStringTopic("Superstructure/CurrentState").publish();
        superStructureDesiredState = table.getStringTopic("Superstructure/DesiredState").publish();
        matchTimePublisher = table.getDoubleTopic("Match Data/MatchTime").publish();
        shotValid = table.getBooleanTopic("Shooter/ShotValid").publish();

        // Swerve Drive State Publishers
        swerveDriveCommand = table.getStringTopic("Swerve Drive State/CurrentCommand").publish();
        swerveDriveCurrentState = table.getStringTopic("Swerve Drive State/CurrentState").publish();
        swerveDriveDesiredState = table.getStringTopic("Swerve Drive State/DesiredState").publish();

        // Shooter publishers
        flywheelAtSetpoint = table.getBooleanTopic("Shooter/FlywheelAtSetpoint").publish();
        hoodAtSetpoint = table.getBooleanTopic("Shooter/HoodAtSetpoint").publish();
        turretAtSetpoint = table.getBooleanTopic("Shooter/TurretAtSetpoint").publish();

        // Initialize Field2d
        NetworkTable fieldTable = table.getSubTable("Field");
        robotField = new Field2d();
        
        // FIX: Now a static field instead of a local variable
        fieldBuilder = new SendableBuilderImpl();
        fieldBuilder.setTable(fieldTable);
        robotField.initSendable(fieldBuilder);
        fieldBuilder.startListeners();
        
        // Initialize swerve drive sendable
        swerveDrive = SwerveDrive.getInstance();
        NetworkTable swerveTable = table.getSubTable("Swerve Drive");
        
        Sendable swerveSendable = new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", 
                    () -> swerveDrive.getSwerveModulePositions()[0].angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", 
                    () -> swerveDrive.getSwerveModuleStates()[0].speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", 
                    () -> swerveDrive.getSwerveModulePositions()[1].angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", 
                    () -> swerveDrive.getSwerveModuleStates()[1].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", 
                    () -> swerveDrive.getSwerveModulePositions()[2].angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", 
                    () -> swerveDrive.getSwerveModuleStates()[2].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", 
                    () -> swerveDrive.getSwerveModulePositions()[3].angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", 
                    () -> swerveDrive.getSwerveModuleStates()[3].speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", 
                    () -> RobotState.getInstance().getEstimatedPose().getRotation().getRadians(), null);
            }
        };
        
        // FIX: Now a static field instead of a local variable
        swerveBuilder = new SendableBuilderImpl();
        swerveBuilder.setTable(swerveTable);
        swerveSendable.initSendable(swerveBuilder);
        swerveBuilder.startListeners();

        // Auto chooser
        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("None", new InstantCommand());
        
        NetworkTable autoTable = table.getSubTable("AutoChooser");
        autoBuilder = new SendableBuilderImpl();
        autoBuilder.setTable(autoTable);
        autoChooser.initSendable(autoBuilder);
        autoBuilder.startListeners();
        autoBuilder.update();  

        initialized = true;
    }

    public static void updateData() {
        initialize();
        
        // Superstructure state logging
        Superstructure superstructure = Superstructure.getInstance();   
        superstructureCurrentState.set(superstructure.getCurrentSystemState().name());
        superStructureDesiredState.set(superstructure.getDesiredSystemState().name()); 

        // Swerve state logging
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        swerveDriveCommand.set(swerveDrive.getCurrentCommand() == null ? "None" : swerveDrive.getCurrentCommand().getName());
        swerveDriveCurrentState.set(swerveDrive.getCurrentSystemState().name());
        swerveDriveDesiredState.set(swerveDrive.getDesiredSystemState().name());

        // Shooter state logging
        Shooter shooter = Shooter.getInstance();
        flywheelAtSetpoint.set(shooter.isFlywheelAtSetpoint());
        hoodAtSetpoint.set(shooter.isHoodAtSetpoint());
        turretAtSetpoint.set(shooter.isTurretAtSetpoint());
        shotValid.set(superstructure.getCurrentSystemState() == CurrentSystemState.READY_FOR_SHOT);

        // Update match data
        matchTimePublisher.set(DriverStation.getMatchTime());

        // Update the field with current robot pose
        robotField.setRobotPose(RobotState.getInstance().getEstimatedPose());

        // FIX: Call update() on all builders so their data is pushed to NetworkTables each loop.
        // Previously only autoBuilder.update() was called — fieldBuilder and swerveBuilder
        // were local variables in initialize() and could never be updated, so the pose
        // and swerve widget data never changed after initialization.
        if (fieldBuilder != null) {
            fieldBuilder.update();
        }
        if (swerveBuilder != null) {
            swerveBuilder.update();
        }
        if (autoBuilder != null) {
            autoBuilder.update();
        }
    }   
}