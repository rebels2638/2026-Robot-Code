package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SystemsMonitor extends SubsystemBase {
    private static SystemsMonitor instance = null;

    public static SystemsMonitor getInstance() {
        if (instance == null) {
            instance = new SystemsMonitor();
        }
        return instance;
    }

    private final SwerveDrive swerve = SwerveDrive.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Hopper hopper = Hopper.getInstance();

    private boolean intakeWasSlipping = false;
    private boolean hopperWasSlipping = false;

    // Add with fields at top
    private static final String AUTO_SHUTOFF_KEY = "Systems/AutoShutoffEnabled";

    private SystemsMonitor() {
        // Set default — starts as alert-only
        SmartDashboard.putBoolean(AUTO_SHUTOFF_KEY, false);
    }

    @Override
    public void periodic() {
        // ── DRIVETRAIN ──
        putStatus("Systems/Drivetrain/Gyro",            swerve.isGyroConnected());
        putStatus("Systems/Drivetrain/FL_Drive",        swerve.isFrontLeftDriveMotorConnected());
        putStatus("Systems/Drivetrain/FL_Steer",        swerve.isFrontLeftSteerMotorConnected());
        putStatus("Systems/Drivetrain/FL_Encoder",      swerve.isFrontLeftSteerEncoderConnected());
        putStatus("Systems/Drivetrain/FR_Drive",        swerve.isFrontRightDriveMotorConnected());
        putStatus("Systems/Drivetrain/FR_Steer",        swerve.isFrontRightSteerMotorConnected());
        putStatus("Systems/Drivetrain/FR_Encoder",      swerve.isFrontRightSteerEncoderConnected());
        putStatus("Systems/Drivetrain/BL_Drive",        swerve.isBackLeftDriveMotorConnected());
        putStatus("Systems/Drivetrain/BL_Steer",        swerve.isBackLeftSteerMotorConnected());
        putStatus("Systems/Drivetrain/BL_Encoder",      swerve.isBackLeftSteerEncoderConnected());
        putStatus("Systems/Drivetrain/BR_Drive",        swerve.isBackRightDriveMotorConnected());
        putStatus("Systems/Drivetrain/BR_Steer",        swerve.isBackRightSteerMotorConnected());
        putStatus("Systems/Drivetrain/BR_Encoder",      swerve.isBackRightSteerEncoderConnected());

        // ── SHOOTER ──
        putStatus("Systems/Shooter/Hood_Motor",         shooter.isHoodMotorConnected());
        putStatus("Systems/Shooter/Turret_Motor",       shooter.isTurretMotorConnected());
        putStatus("Systems/Shooter/Flywheel_Motor",     shooter.isFlywheelMotorConnected());
        putStatus("Systems/Shooter/Flywheel_Follower",  shooter.isFlywheelFollowerMotorConnected());

        // ── INTAKE ──
        putStatus("Systems/Intake/Roller_Motor",        intake.isRollerMotorConnected());
        putStatus("Systems/Intake/Pivot_Motor",         intake.isPivotMotorConnected());
        putStatus("Systems/Intake/Belt_OK",             !intake.isBeltSlipping());

        // ── HOPPER ──
        putStatus("Systems/Hopper/Motor",               hopper.isHopperMotorConnected());
        putStatus("Systems/Hopper/Belt_OK",             !hopper.isBeltSlipping());

        // ── OVERALL ──
        boolean allOk = swerve.isGyroConnected()
            && swerve.isFrontLeftDriveMotorConnected()
            && swerve.isFrontRightDriveMotorConnected()
            && swerve.isBackLeftDriveMotorConnected()
            && swerve.isBackRightDriveMotorConnected()
            && shooter.isHoodMotorConnected()
            && shooter.isTurretMotorConnected()
            && shooter.isFlywheelMotorConnected()
            && intake.isRollerMotorConnected()
            && intake.isPivotMotorConnected()
            && hopper.isHopperMotorConnected()
            && !intake.isBeltSlipping()
            && !hopper.isBeltSlipping();

        SmartDashboard.putBoolean("Systems/ROBOT_OK", allOk);
        SmartDashboard.putString("Systems/STATUS", allOk ? "ALL SYSTEMS GO" : "CHECK ALERTS");

        boolean autoShutoff = SmartDashboard.getBoolean(AUTO_SHUTOFF_KEY, false);
        boolean intakeSlipping = intake.isBeltSlipping();
        boolean hopperSlipping = hopper.isBeltSlipping();

        if (autoShutoff) {
            // Only call on the transition, not every loop
            if (intakeSlipping && !intakeWasSlipping) {
                intake.enableRollerEStop();
                intake.enablePivotEStop();
            } else if (!intakeSlipping && intakeWasSlipping) {
                intake.disableRollerEStop();
                intake.disablePivotEStop();
            }

            if (hopperSlipping && !hopperWasSlipping) {
                hopper.enableEStop();
            } else if (!hopperSlipping && hopperWasSlipping) {
                hopper.disableEStop();
            }
        } else {
            // AutoShutoff just got toggled off — make sure nothing is stuck disabled
            if (intakeWasSlipping) {
                intake.disableRollerEStop();
                intake.disablePivotEStop();
            }
            if (hopperWasSlipping) {
                hopper.disableEStop();
            }
        }

        intakeWasSlipping = intakeSlipping;
        hopperWasSlipping = hopperSlipping;
    }

    private void putStatus(String key, boolean ok) {
        SmartDashboard.putBoolean(key, ok);
    }
}