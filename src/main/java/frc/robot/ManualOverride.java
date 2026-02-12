package frc.robot;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ManualOverride {
    private static Shooter shooter;
    private static NetworkTableEntry shooterToggle;
    private static NetworkTableEntry hoodToggle;
    private static NetworkTableEntry turretToggle;
    private static NetworkTableEntry flywheelToggle;
    private static boolean shooterEStopEnabled = false;
    private static boolean hoodEStopEnabled = false;
    private static boolean turretEStopEnabled = false;
    private static boolean flywheelEStopEnabled = false;
    private static NetworkTableEntry[] shooterToggles;
    private static boolean[] shooterEStopsEnabled;

    private static Intake intake;
    private static NetworkTableEntry intakeToggle;
    private static NetworkTableEntry rollerToggle;
    private static NetworkTableEntry pivotToggle;
    private static boolean intakeEStopEnabled = false;
    private static boolean rollerEStopEnabled = false;
    private static boolean pivotEStopEnabled = false;
    private static NetworkTableEntry[] intakeToggles;
    private static boolean[] intakeEStopsEnabled;

    private static Kicker kicker;
    private static NetworkTableEntry kickerToggle;
    private static boolean kickerEStopEnabled = false;

    private static Hopper hopper;
    private static NetworkTableEntry hopperToggle;
    private static boolean hopperEStopEnabled = false;

    private static Climber climber;
    private static NetworkTableEntry climberToggle;
    private static boolean climberEStopEnabled = false;

    // Swerve Drive set up
    // SwerveDrive toggle -> Module toggles -> Steer/Drive toggles
    private static SwerveDrive swerveDrive;
    private static NetworkTableEntry swerveDriveToggle;
    private static boolean swerveDriveEStopEnabled = false;
    
    private static class SwerveModuleOverride {
        NetworkTableEntry moduleToggle;
        NetworkTableEntry driveToggle;
        NetworkTableEntry steerToggle;
        boolean moduleEStopEnabled = false;
        boolean driveEStopEnabled = false;
        boolean steerEStopEnabled = false;
        
        SwerveModuleOverride(NetworkTable table, String moduleName) {
            String path = "Swerve/" + moduleName + "/";
            moduleToggle = table.getEntry(path + "Disable" + moduleName);
            driveToggle = table.getEntry(path + "Disable" + moduleName + "Drive");
            steerToggle = table.getEntry(path + "Disable" + moduleName + "Steer");
            
            moduleToggle.setBoolean(false);
            driveToggle.setBoolean(false);
            steerToggle.setBoolean(false);
        }
    }

    public enum SwerveModuleNames {
        FRONT_LEFT("FrontLeft"),
        FRONT_RIGHT("FrontRight"),
        BACK_LEFT("BackLeft"),
        BACK_RIGHT("BackRight");
        
        public final String name;
        SwerveModuleNames(String name) { this.name = name; }
    }

    private static Map<SwerveModuleNames, SwerveModuleOverride> swerveModules;


    private static boolean initalized = false;

    private static void initalize() {
        if (initalized) return;
        initalized = true;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("ManualOverride");

        shooter = Shooter.getInstance();
        shooterToggle = table.getEntry("Shooter/DisableShooter");
        hoodToggle = table.getEntry("Shooter/DisableShooterHood");
        turretToggle = table.getEntry("Shooter/DisableShooterTurret");
        flywheelToggle = table.getEntry("Shooter/DiableShooterFlywheel");
        shooterToggle.setBoolean(false);
        hoodToggle.setBoolean(false);
        turretToggle.setBoolean(false);
        flywheelToggle.setBoolean(false);

        shooterToggles = new NetworkTableEntry[]{hoodToggle, turretToggle, flywheelToggle};
        shooterEStopsEnabled = new boolean[]{hoodEStopEnabled, turretEStopEnabled, flywheelEStopEnabled};

        kicker = Kicker.getInstance();
        kickerToggle = table.getEntry("DisableKicker");
        kickerToggle.setBoolean(false);

        intake = Intake.getInstance();
        intakeToggle = table.getEntry("Intake/DisableIntake");
        rollerToggle = table.getEntry("Intake/DisableIntakeRoller");
        pivotToggle = table.getEntry("Intake/DisableIntakePivot");
        intakeToggle.setBoolean(false);
        rollerToggle.setBoolean(false);
        pivotToggle.setBoolean(false);

        intakeToggles = new NetworkTableEntry[]{rollerToggle, pivotToggle};
        intakeEStopsEnabled = new boolean[]{rollerEStopEnabled, pivotEStopEnabled};

        hopper = Hopper.getInstance();
        hopperToggle = table.getEntry("DisableHopper");
        hopperToggle.setBoolean(false);

        climber = Climber.getInstance();
        climberToggle = table.getEntry("DisableClimber");
        climberToggle.setBoolean(false);

        // Swerve Drive
        swerveDrive = SwerveDrive.getInstance();
        swerveDriveToggle = table.getEntry("Swerve/DisableSwerveDrive");
        swerveDriveToggle.setBoolean(false);

        swerveModules = new EnumMap<>(SwerveModuleNames.class);
        for (SwerveModuleNames module : SwerveModuleNames.values()) {
            swerveModules.put(module, new SwerveModuleOverride(table, module.name));
        }
    }   

    public static void update() {
        initalize();

        // ===== SHOOTER CASCADING LOGIC =====
        boolean shooterToggleState = shooterToggle.getValue().getBoolean();
        
        // If main shooter toggle turns ON, enable all sub-components
        if (shooterToggleState && !shooterEStopEnabled) {
            shooter.enableHoodEStop();
            shooter.enableTurretEStop();
            shooter.enableFlywheelEStop();
            hoodToggle.setBoolean(true);
            turretToggle.setBoolean(true);
            flywheelToggle.setBoolean(true);
            hoodEStopEnabled = true;
            turretEStopEnabled = true;
            flywheelEStopEnabled = true;
            shooterEStopEnabled = true;
        }
        // If main shooter toggle turns OFF, disable all sub-components
        else if (!shooterToggleState && shooterEStopEnabled) {
            shooter.disableHoodEStop();
            shooter.disableTurretEStop();
            shooter.disableFlywheelEStop();
            hoodToggle.setBoolean(false);
            turretToggle.setBoolean(false);
            flywheelToggle.setBoolean(false);
            hoodEStopEnabled = false;
            turretEStopEnabled = false;
            flywheelEStopEnabled = false;
            shooterEStopEnabled = false;       
        }
        // Handle individual motor toggles
        else {
            // Hood toggle
            boolean hoodToggleState = hoodToggle.getValue().getBoolean();
            if (hoodToggleState && !hoodEStopEnabled) {
                shooter.enableHoodEStop();
                hoodEStopEnabled = true;
            }
            else if (!hoodToggleState && hoodEStopEnabled) {
                shooter.disableHoodEStop();
                hoodEStopEnabled = false;
            }

            // Turret toggle
            boolean turretToggleState = turretToggle.getValue().getBoolean();
            if (turretToggleState && !turretEStopEnabled) {
                shooter.enableTurretEStop();
                turretEStopEnabled = true;
            }
            else if (!turretToggleState && turretEStopEnabled) {
                shooter.disableTurretEStop();
                turretEStopEnabled = false;
            }

            // Flywheel toggle
            boolean flywheelToggleState = flywheelToggle.getValue().getBoolean();
            if (flywheelToggleState && !flywheelEStopEnabled) {
                shooter.enableFlywheelEStop();
                flywheelEStopEnabled = true;
            }
            else if (!flywheelToggleState && flywheelEStopEnabled) {
                shooter.disableFlywheelEStop();
                flywheelEStopEnabled = false;
            }

            // Update main toggle based on individual states
            boolean allShooterMotorsEnabled = hoodEStopEnabled && turretEStopEnabled && flywheelEStopEnabled;
            if (allShooterMotorsEnabled && !shooterEStopEnabled) {
                shooterToggle.setBoolean(true);
                shooterEStopEnabled = true;
            }
            else if (!allShooterMotorsEnabled && shooterEStopEnabled) {
                shooterToggle.setBoolean(false);
                shooterEStopEnabled = false;
            }
        }

        // ===== INTAKE CASCADING LOGIC =====
        boolean intakeToggleState = intakeToggle.getValue().getBoolean();
        
        // If main intake toggle turns ON, enable all sub-components
        if (intakeToggleState && !intakeEStopEnabled) {
            intake.enablePivotEStop();
            intake.enableRollerEStop();
            pivotToggle.setBoolean(true);
            rollerToggle.setBoolean(true);
            pivotEStopEnabled = true;
            rollerEStopEnabled = true;
            intakeEStopEnabled = true;
        }
        // If main intake toggle turns OFF, disable all sub-components
        else if (!intakeToggleState && intakeEStopEnabled) {
            intake.disablePivotEStop();
            intake.disableRollerEStop();
            pivotToggle.setBoolean(false);
            rollerToggle.setBoolean(false);
            pivotEStopEnabled = false;
            rollerEStopEnabled = false;
            intakeEStopEnabled = false;       
        }
        // Handle individual motor toggles
        else {
            // Pivot toggle
            boolean pivotToggleState = pivotToggle.getValue().getBoolean();
            if (pivotToggleState && !pivotEStopEnabled) {
                intake.enablePivotEStop();
                pivotEStopEnabled = true;
            }
            else if (!pivotToggleState && pivotEStopEnabled) {
                intake.disablePivotEStop();
                pivotEStopEnabled = false;
            }

            // Roller toggle
            boolean rollerToggleState = rollerToggle.getValue().getBoolean();
            if (rollerToggleState && !rollerEStopEnabled) {
                intake.enableRollerEStop();
                rollerEStopEnabled = true;
            }
            else if (!rollerToggleState && rollerEStopEnabled) {
                intake.disableRollerEStop();
                rollerEStopEnabled = false;
            }

            // Update main toggle based on individual states
            boolean allIntakeMotorsEnabled = pivotEStopEnabled && rollerEStopEnabled;
            if (allIntakeMotorsEnabled && !intakeEStopEnabled) {
                intakeToggle.setBoolean(true);
                intakeEStopEnabled = true;
            }
            else if (!allIntakeMotorsEnabled && intakeEStopEnabled) {
                intakeToggle.setBoolean(false);
                intakeEStopEnabled = false;
            }
        }

        // Kicker
        NetworkTableValue kickerToggleState = kickerToggle.getValue();
        if (kickerToggleState.getBoolean()) {
            kicker.enableEStop();
            kickerEStopEnabled = true;
        }
        else if (!kickerToggleState.getBoolean() && kickerEStopEnabled) {
            kicker.disableEStop();
            kickerEStopEnabled = false;       
        }

        // Hopper
        NetworkTableValue hopperToggleState = hopperToggle.getValue();
        if (hopperToggleState.getBoolean()) {
            hopper.enableEStop();
            hopperEStopEnabled = true;
        }
        else if (!hopperToggleState.getBoolean() && hopperEStopEnabled) {
            hopper.disableEStop();
            hopperEStopEnabled = false;       
        }

        // Climber
        NetworkTableValue climberToggleState = climberToggle.getValue();
        if (climberToggleState.getBoolean()) {
            climber.enableEStop();
            climberEStopEnabled = true;
        }
        else if (!climberToggleState.getBoolean() && climberEStopEnabled) {
            climber.disableEStop();
            climberEStopEnabled = false;       
        }

        // Swerve Drive
        boolean swerveDriveToggleState = swerveDriveToggle.getValue().getBoolean();

        if (swerveDriveToggleState && !swerveDriveEStopEnabled) {
            // Master toggle just turned ON - enable all modules
            for (Map.Entry<SwerveModuleNames, SwerveModuleOverride> entry : swerveModules.entrySet()) {
                SwerveModuleOverride module = entry.getValue();
                
                // Enable estops based on module position
                switch (entry.getKey()) {
                    case FRONT_LEFT:
                        swerveDrive.enableFrontLeftSteerEStop();
                        swerveDrive.enableFrontLeftDriveEStop();
                        break;
                    case FRONT_RIGHT:
                        swerveDrive.enableFrontRightSteerEStop();
                        swerveDrive.enableFrontRightDriveEStop();
                        break;
                    case BACK_LEFT:
                        swerveDrive.enableBackLeftSteerEStop();
                        swerveDrive.enableBackLeftDriveEStop();
                        break;
                    case BACK_RIGHT:
                        swerveDrive.enableBackRightSteerEStop();
                        swerveDrive.enableBackRightDriveEStop();
                        break;
                }
                
                // Update module state
                module.driveToggle.setBoolean(true);
                module.steerToggle.setBoolean(true);
                module.moduleToggle.setBoolean(true);
                module.driveEStopEnabled = true;
                module.steerEStopEnabled = true;
                module.moduleEStopEnabled = true;
            }
            swerveDriveEStopEnabled = true;
        }
        else if (!swerveDriveToggleState && swerveDriveEStopEnabled) {
            // Master toggle just turned OFF - disable all modules
            for (Map.Entry<SwerveModuleNames, SwerveModuleOverride> entry : swerveModules.entrySet()) {
                SwerveModuleOverride module = entry.getValue();
                
                // Disable estops based on module position
                switch (entry.getKey()) {
                    case FRONT_LEFT:
                        swerveDrive.disableFrontLeftSteerEStop();
                        swerveDrive.disableFrontLeftDriveEStop();
                        break;
                    case FRONT_RIGHT:
                        swerveDrive.disableFrontRightSteerEStop();
                        swerveDrive.disableFrontRightDriveEStop();
                        break;
                    case BACK_LEFT:
                        swerveDrive.disableBackLeftSteerEStop();
                        swerveDrive.disableBackLeftDriveEStop();
                        break;
                    case BACK_RIGHT:
                        swerveDrive.disableBackRightSteerEStop();
                        swerveDrive.disableBackRightDriveEStop();
                        break;
                }
                
                // Update module state
                module.driveToggle.setBoolean(false);
                module.steerToggle.setBoolean(false);
                module.moduleToggle.setBoolean(false);
                module.driveEStopEnabled = false;
                module.steerEStopEnabled = false;
                module.moduleEStopEnabled = false;
            }
            swerveDriveEStopEnabled = false;
        }

        // Individual module toggles
        for (Map.Entry<SwerveModuleNames, SwerveModuleOverride> entry : swerveModules.entrySet()) {
            SwerveModuleOverride module = entry.getValue();
            boolean moduleToggleState = module.moduleToggle.getValue().getBoolean();

            if (moduleToggleState && !module.moduleEStopEnabled) {
                // Enable this specific module
                switch (entry.getKey()) {
                    case FRONT_LEFT:
                        swerveDrive.enableFrontLeftSteerEStop();
                        swerveDrive.enableFrontLeftDriveEStop();
                        break;
                    case FRONT_RIGHT:
                        swerveDrive.enableFrontRightSteerEStop();
                        swerveDrive.enableFrontRightDriveEStop();
                        break;
                    case BACK_LEFT:
                        swerveDrive.enableBackLeftSteerEStop();
                        swerveDrive.enableBackLeftDriveEStop();
                        break;
                    case BACK_RIGHT:
                        swerveDrive.enableBackRightSteerEStop();
                        swerveDrive.enableBackRightDriveEStop();
                        break;
                }
                module.driveToggle.setBoolean(true);
                module.steerToggle.setBoolean(true);
                module.driveEStopEnabled = true;
                module.steerEStopEnabled = true;
                module.moduleEStopEnabled = true;
            }
            else if (!moduleToggleState && module.moduleEStopEnabled) {
                // Disable this specific module
                switch (entry.getKey()) {
                    case FRONT_LEFT:
                        swerveDrive.disableFrontLeftSteerEStop();
                        swerveDrive.disableFrontLeftDriveEStop();
                        break;
                    case FRONT_RIGHT:
                        swerveDrive.disableFrontRightSteerEStop();
                        swerveDrive.disableFrontRightDriveEStop();
                        break;
                    case BACK_LEFT:
                        swerveDrive.disableBackLeftSteerEStop();
                        swerveDrive.disableBackLeftDriveEStop();
                        break;
                    case BACK_RIGHT:
                        swerveDrive.disableBackRightSteerEStop();
                        swerveDrive.disableBackRightDriveEStop();
                        break;
                }
                module.driveToggle.setBoolean(false);
                module.steerToggle.setBoolean(false);
                module.driveEStopEnabled = false;
                module.steerEStopEnabled = false;
                module.moduleEStopEnabled = false;
            }
        }

        // Individual motor toggles (drive/steer)
        for (Map.Entry<SwerveModuleNames, SwerveModuleOverride> entry : swerveModules.entrySet()) {
            SwerveModuleOverride module = entry.getValue();
            boolean driveToggleState = module.driveToggle.getValue().getBoolean();
            boolean steerToggleState = module.steerToggle.getValue().getBoolean();

            // Handle drive motor toggle
            if (driveToggleState && !module.driveEStopEnabled) {
                switch (entry.getKey()) {
                    case FRONT_LEFT:
                        swerveDrive.enableFrontLeftDriveEStop();
                        break;
                    case FRONT_RIGHT:
                        swerveDrive.enableFrontRightDriveEStop();
                        break;
                    case BACK_LEFT:
                        swerveDrive.enableBackLeftDriveEStop();
                        break;
                    case BACK_RIGHT:
                        swerveDrive.enableBackRightDriveEStop();
                        break;
                }
                module.driveEStopEnabled = true;
            }
            else if (!driveToggleState && module.driveEStopEnabled) {
                switch (entry.getKey()) {
                    case FRONT_LEFT:
                        swerveDrive.disableFrontLeftDriveEStop();
                        break;
                    case FRONT_RIGHT:
                        swerveDrive.disableFrontRightDriveEStop();
                        break;
                    case BACK_LEFT:
                        swerveDrive.disableBackLeftDriveEStop();
                        break;
                    case BACK_RIGHT:
                        swerveDrive.disableBackRightDriveEStop();
                        break;
                }
                module.driveEStopEnabled = false;
            }

            // Handle steer motor toggle
            if (steerToggleState && !module.steerEStopEnabled) {
                switch (entry.getKey()) {
                    case FRONT_LEFT:
                        swerveDrive.enableFrontLeftSteerEStop();
                        break;
                    case FRONT_RIGHT:
                        swerveDrive.enableFrontRightSteerEStop();
                        break;
                    case BACK_LEFT:
                        swerveDrive.enableBackLeftSteerEStop();
                        break;
                    case BACK_RIGHT:
                        swerveDrive.enableBackRightSteerEStop();
                        break;
                }
                module.steerEStopEnabled = true;
            }
            else if (!steerToggleState && module.steerEStopEnabled) {
                switch (entry.getKey()) {
                    case FRONT_LEFT:
                        swerveDrive.disableFrontLeftSteerEStop();
                        break;
                    case FRONT_RIGHT:
                        swerveDrive.disableFrontRightSteerEStop();
                        break;
                    case BACK_LEFT:
                        swerveDrive.disableBackLeftSteerEStop();
                        break;
                    case BACK_RIGHT:
                        swerveDrive.disableBackRightSteerEStop();
                        break;
                }
                module.steerEStopEnabled = false;
            }

            // Update module toggle based on motor states
            boolean bothMotorsEnabled = module.driveEStopEnabled && module.steerEStopEnabled;
            if (bothMotorsEnabled && !module.moduleEStopEnabled) {
                module.moduleToggle.setBoolean(true);
                module.moduleEStopEnabled = true;
            }
            else if (!bothMotorsEnabled && module.moduleEStopEnabled) {
                module.moduleToggle.setBoolean(false);
                module.moduleEStopEnabled = false;
            }
        }

        // Update master toggle based on all module states
        boolean allModulesEnabled = true;
        for (SwerveModuleOverride module : swerveModules.values()) {
            if (!module.moduleEStopEnabled) {
                allModulesEnabled = false;
                break;
            }
        }

        if (allModulesEnabled && !swerveDriveEStopEnabled) {
            swerveDriveToggle.setBoolean(true);
            swerveDriveEStopEnabled = true;
        }
        else if (!allModulesEnabled && swerveDriveEStopEnabled) {
            swerveDriveToggle.setBoolean(false);
            swerveDriveEStopEnabled = false;
        }
    }
}