package frc.robot.commands.check;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperSetpoint;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.Kicker.KickerSetpoint;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.FlywheelSetpoint;
import frc.robot.subsystems.shooter.Shooter.HoodSetpoint;
import frc.robot.subsystems.shooter.Shooter.TurretSetpoint;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.module.ModuleIOInputsAutoLogged;

/**
 * Checks the functionality of all subsystems on the robot.<br>
 * <br>
 * The checks are as follows:<br>
 * 1. Swerve Drive: Moves the robot forward at 1 m/s for 2 seconds and checks
 * if the drive velocity is roughly 1 m/s.<br>
 * 2. Shooter: Sets the flywheel to dynamic, hood to home, and turret to home.
 * Then checks if the flywheel velocity is roughly 40 RPS.<br>
 * 3. Hopper/Kicker: Sets the hopper and kicker to feeding and then checks if
 * they are running.<br>
 */
public class CheckSystemCommand extends SequentialCommandGroup {
    private static final String LOG_ROOT = "SystemCheck/";

    public CheckSystemCommand() {
        SwerveDrive swerve = SwerveDrive.getInstance();
        Shooter shooter = Shooter.getInstance();
        Hopper hopper = Hopper.getInstance();
        Kicker kicker = Kicker.getInstance();

        addRequirements(swerve, shooter, hopper, kicker);

        addCommands(
                logStep("Starting System Check"),

                logStep("Checking Swerve Drive"),
                new FunctionalCommand(
                        () -> {
                        },
                        () -> swerve.driveRobotRelative(new ChassisSpeeds(1.0, 0.0, 0.0)),
                        (interrupted) -> swerve.driveRobotRelative(new ChassisSpeeds()),
                        () -> false,
                        swerve).withTimeout(2.0),

                Commands.runOnce(() -> {
                    boolean allGood = true;

                    ModuleIOInputsAutoLogged[] inputs = swerve.getModuleInputs();
                    double tolerance = 0.1;

                    double sumAbsVel = 0.0;
                    for (int i = 0; i < 4; i++) {
                        double velocity = inputs[i].driveVelocityMetersPerSec;
                        sumAbsVel += Math.abs(velocity);

                        boolean velPass = Math.abs(velocity - 1.0) < tolerance && velocity > 0.0;
                        check("SwerveModule" + i + "_DriveVelocity", velPass);
                        allGood &= velPass;
                    }

                    double avgAbsVel = sumAbsVel / 4.0;
                    boolean avgPass = Math.abs(avgAbsVel - 1.0) < tolerance;
                    check("Swerve_AvgAbsDriveVelocity", avgPass);
                    allGood &= avgPass;

                    check("Swerve_Drive_Overall", allGood);
                }),

                logStep("Checking Shooter"),
                Commands.runOnce(() -> {
                    shooter.setFlywheelRPSSupplier(() -> 40.0);
                    shooter.setFlywheelSetpoint(FlywheelSetpoint.DYNAMIC);

                    shooter.setHoodSetpoint(HoodSetpoint.HOME);
                    shooter.setTurretSetpoint(TurretSetpoint.HOME);
                }, shooter),

                Commands.waitSeconds(1.5),

                Commands.runOnce(() -> {
                    double velocity = shooter.getFlywheelVelocityRotationsPerSec();
                    check("Flywheel_SpinUp", velocity > 35.0);

                    shooter.setFlywheelSetpoint(FlywheelSetpoint.OFF);
                }, shooter),

                logStep("Checking Hopper/Kicker"),
                Commands.runOnce(() -> {
                    hopper.setSetpoint(HopperSetpoint.FEEDING);
                    kicker.setSetpoint(KickerSetpoint.FEEDING);
                }, hopper, kicker),

                Commands.waitSeconds(1.0),

                Commands.runOnce(() -> {
                    hopper.setSetpoint(HopperSetpoint.OFF);
                    kicker.setSetpoint(KickerSetpoint.OFF);
                }, hopper, kicker),

                logStep("System Check Complete"));
    }

    private Command logStep(String stepName) {
        return Commands.runOnce(() -> Logger.recordOutput(LOG_ROOT + "CurrentStep", stepName));
    }

    private void check(String name, boolean condition) {
        Logger.recordOutput(LOG_ROOT + "Checks/" + name, condition ? "PASS" : "FAIL");
    }
}
