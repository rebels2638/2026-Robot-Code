package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Dashboard {
    private static Field2d robotField;
    private static SwerveDrive swerveDrive;

    private static StringPublisher superstructureCurrentState;
    private static StringPublisher superStructureDesiredState;
    
    private static boolean initialized = false;

    private static void initialize() {
        if (initialized) return;
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("Dashboard");

        // Create publishers once
        superstructureCurrentState = table.getStringTopic("Superstructure/Current State").publish();
        superStructureDesiredState = table.getStringTopic("Superstructure/Desired State").publish();

        // Initialize Field2d
        NetworkTable fieldTable = table.getSubTable("Field");
        robotField = new Field2d();
        
        SendableBuilderImpl fieldBuilder = new SendableBuilderImpl();
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
                    () -> RobotState.getInstance().getAngleToFloor().getRadians(), null);
            }
        };
        
        SendableBuilderImpl swerveBuilder = new SendableBuilderImpl();
        swerveBuilder.setTable(swerveTable);
        swerveSendable.initSendable(swerveBuilder);
        swerveBuilder.startListeners();

        initialized = true;
    }

    public static void updateData() {
        initialize(); // Initialize on first call
        
        // Superstructure state logging
        Superstructure superstructure = Superstructure.getInstance();   
        superstructureCurrentState.set(superstructure.getCurrentState().name());
        superStructureDesiredState.set(superstructure.getDesiredState().name());

        // Update the field with current robot pose
        robotField.setRobotPose(RobotState.getInstance().getEstimatedPose());
    }   
}
