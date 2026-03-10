package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.configs.ShooterConfig;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Shooter extends SubsystemBase {
    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    // FSM State Enums
    public enum ShooterCurrentState {
        DISABLED,
        HOME,
        TRACKING,
        PREPARING_FOR_SHOT,
        READY_FOR_SHOT,
        SHOOTING
    }

    public enum ShooterDesiredState {
        DISABLED,
        HOME,
        TRACKING,
        READY_FOR_SHOT,
        SHOOTING
    }

    // FSM State Variables
    private ShooterCurrentState currentState = ShooterCurrentState.DISABLED;
    private ShooterDesiredState desiredState = ShooterDesiredState.DISABLED;

    // Setpoint Suppliers (set by Superstructure)
    private Supplier<Rotation2d> hoodAngleSupplier = () -> Rotation2d.fromDegrees(45.0);
    private Supplier<Rotation2d> turretAngleSupplier = () -> new Rotation2d(0);
    private Supplier<Double> flywheelRPSSupplier = () -> 0.0;

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final ShooterConfig config;
    
    private final DashboardMotorControlLoopConfigurator hoodControlLoopConfigurator;
    private final DashboardMotorControlLoopConfigurator turretControlLoopConfigurator;
    private final DashboardMotorControlLoopConfigurator flywheelControlLoopConfigurator;

    private double hoodSetpointRotations = 0.0;
    private double turretSetpointRotations = 0.0;
    private double flywheelSetpointRPS = 0.0;

    private Shooter() {
        config = ConfigLoader.load("shooter", ShooterConfig.class);
        shooterIO = RobotBase.isSimulation() ? new ShooterIOSim(config) : new ShooterIOTalonFX(config);

        hoodControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Shooter/hoodControlLoop", 
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.hoodKP,
                config.hoodKI,
                config.hoodKD,
                config.hoodKS,
                config.hoodKV,
                config.hoodKA
            )
        );
        turretControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Shooter/turretControlLoop", 
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.turretKP,
                config.turretKI,
                config.turretKD,
                config.turretKS,
                config.turretKV,
                config.turretKA
            )
        );
        flywheelControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Shooter/flywheelControlLoop", 
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.flywheelKP,
                config.flywheelKI,
                config.flywheelKD,
                config.flywheelKS,
                config.flywheelKV,
                config.flywheelKA
            )
        );
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);

        // Handle control loop configuration updates
        if (hoodControlLoopConfigurator.hasChanged()) {
            shooterIO.configureHoodControlLoop(hoodControlLoopConfigurator.getConfig());
        }
        if (turretControlLoopConfigurator.hasChanged()) {
            shooterIO.configureTurretControlLoop(turretControlLoopConfigurator.getConfig());
        }
        if (flywheelControlLoopConfigurator.hasChanged()) {
            shooterIO.configureFlywheelControlLoop(flywheelControlLoopConfigurator.getConfig());
        }

        // FSM processing
        handleStateTransitions();
        handleCurrentState();
    }

    /**
     * Determines the next measured state based on the desired state.
     * Handles transitions between states with proper validation.
     */
    private void handleStateTransitions() {
        switch (desiredState) {
            case DISABLED:
                currentState = ShooterCurrentState.DISABLED;
                break;

            case HOME:
                currentState = ShooterCurrentState.HOME;
                break;

            case TRACKING:
                currentState = ShooterCurrentState.TRACKING;
                break;

            case READY_FOR_SHOT:
                // READY_FOR_SHOT requires mechanisms to be at setpoints
                // Otherwise we're in PREPARING_FOR_SHOT
                if (isReadyForShot()) {
                    currentState = ShooterCurrentState.READY_FOR_SHOT;
                } else {
                    currentState = ShooterCurrentState.PREPARING_FOR_SHOT;
                }
                break;

            case SHOOTING:
                // SHOOTING only allowed when we're in READY_FOR_SHOT
                if (currentState == ShooterCurrentState.READY_FOR_SHOT) {
                    currentState = ShooterCurrentState.SHOOTING;
                } else if (currentState != ShooterCurrentState.SHOOTING) {
                    currentState = ShooterCurrentState.PREPARING_FOR_SHOT;
                }
                // Otherwise we're in SHOOTING, and allow superstructure to handle the transition to READY_FOR_SHOT
                break;
        }
    }

    /**
     * Executes behavior for the current state - applies setpoints from suppliers.
     */
    private void handleCurrentState() {
        switch (currentState) {
            case DISABLED:
                handleDISABLEDState();
                break;
            case HOME:
                handleHomeState();
                break;
            case TRACKING:
                handleTrackingState();
                break;
            case PREPARING_FOR_SHOT:
                handlePreparingForShotState();
                break;
            case READY_FOR_SHOT:
                handleReadyForShotState();
                break;
            case SHOOTING:
                handleShootingState();
                break;
        }
    }

    private void handleDISABLEDState() {
        // All motors DISABLED/coast
        setShotVelocity(0);
        // Keep current hood/turret position or go to default
        setHoodAngle(Rotation2d.fromDegrees(45.0));
        setTurretAngle(new Rotation2d(Math.PI));
    }

    private void handleHomeState() {
        // Idle position, no spin
        setShotVelocity(0);
        setHoodAngle(Rotation2d.fromDegrees(45.0));
        setTurretAngle(new Rotation2d(Math.PI));
    }

    private void handleTrackingState() {
        // Track target with hood/turret but don't spin up flywheel
        setShotVelocity(0);
        setHoodAngle(hoodAngleSupplier.get());
        setTurretAngle(turretAngleSupplier.get());
    }

    private void handlePreparingForShotState() {
        // Spin up flywheel and track target
        setShotVelocity(flywheelRPSSupplier.get());
        setHoodAngle(hoodAngleSupplier.get());
        setTurretAngle(turretAngleSupplier.get());
    }

    private void handleReadyForShotState() {
        // Maintain ready state
        setShotVelocity(flywheelRPSSupplier.get());
        setHoodAngle(hoodAngleSupplier.get());
        setTurretAngle(turretAngleSupplier.get());
    }

    private void handleShootingState() {
        // Fire the shot - shooter maintains flywheel/hood/turret, kicker handles feeding
        setShotVelocity(flywheelRPSSupplier.get());
        setHoodAngle(hoodAngleSupplier.get());
        setTurretAngle(turretAngleSupplier.get());
    }

    /**
     * Checks if all mechanisms are at their setpoints and ready to shoot.
     */
    private boolean isReadyForShot() {
        return isFlywheelAtSetpoint() && 
               isHoodAtSetpoint() && 
               isTurretAtSetpoint();
    }

    // Supplier setters (called by Superstructure)
    public void setHoodAngleSupplier(Supplier<Rotation2d> supplier) {
        this.hoodAngleSupplier = supplier;
    }

    public void setTurretAngleSupplier(Supplier<Rotation2d> supplier) {
        this.turretAngleSupplier = supplier;
    }

    public void setFlywheelRPSSupplier(Supplier<Double> supplier) {
        this.flywheelRPSSupplier = supplier;
    }

    // State getters/setters
    @AutoLogOutput(key = "Shooter/currentState")
    public ShooterCurrentState getCurrentState() {
        return currentState;
    }

    @AutoLogOutput(key = "Shooter/desiredState")
    public ShooterDesiredState getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(ShooterDesiredState desiredState) {
        this.desiredState = desiredState;
    }

    // Direct setters for mechanism control (now private, used internally by state handlers)
    private void setHoodAngle(Rotation2d angle) {
        double clampedAngle = MathUtil.clamp(angle.getRotations(), config.hoodMinAngleRotations, config.hoodMaxAngleRotations); 

        hoodSetpointRotations = clampedAngle;
        Logger.recordOutput("Shooter/angleSetpointRotations", clampedAngle);
        Logger.recordOutput("Shooter/rawSetpointRotations", clampedAngle);

        shooterIO.setAngle(clampedAngle);
    }

    private void setTurretAngle(Rotation2d angle) {
        Logger.recordOutput("Shooter/unclampedTurretAngleRotations", angle.getRotations());

        double initAngle = angle.getDegrees();
        double minDeg = config.turretMinAngleDeg;
        double maxDeg = config.turretMaxAngleDeg;
        
        double targetAngleDeg;
        
        // Check if the angle (or wrapped equivalents) are within the valid range
        if (initAngle >= minDeg && initAngle <= maxDeg) {
            targetAngleDeg = initAngle;
        } else if (initAngle + 360.0 >= minDeg && initAngle + 360.0 <= maxDeg) {
            targetAngleDeg = initAngle + 360.0;
        } else if (initAngle - 360.0 >= minDeg && initAngle - 360.0 <= maxDeg) {
            targetAngleDeg = initAngle - 360.0;
        } else {
            // None of the wrapped angles are in range, clamp to nearest bound
            targetAngleDeg = MathUtil.clamp(initAngle, minDeg, maxDeg);
        }

        double clampedAngleRotations = targetAngleDeg / 360.0;
        turretSetpointRotations = clampedAngleRotations;
        Logger.recordOutput("Shooter/turretAngleSetpointRotations", clampedAngleRotations);

        shooterIO.setTurretAngle(clampedAngleRotations);
    }

    private void setShotVelocity(double velocityRotationsPerSec) {
        flywheelSetpointRPS = velocityRotationsPerSec;
        Logger.recordOutput("Shooter/shotVelocitySetpointRotationsPerSec", velocityRotationsPerSec);
        shooterIO.setShotVelocity(velocityRotationsPerSec);
    }

    // Mechanism getters
    public double getHoodAngleRotations() {
        return shooterInputs.hoodAngleRotations;
    }

    public double getTurretAngleRotations() {
        return shooterInputs.turretAngleRotations;
    }

    public double getFlywheelVelocityRotationsPerSec() {
        return shooterInputs.flywheelVelocityRotationsPerSec;
    }

    public double calculateShotExitVelocityMetersPerSec(double flywheelVelocityRotationsPerSec) {
        return flywheelVelocityRotationsPerSec * 2 * Math.PI * config.flywheelRadiusMeters / 2; 
        // divide by 2 because the flywheel is a pulling the ball along the hood radius
    }

    public double getShotExitVelocityMetersPerSec() {
        return calculateShotExitVelocityMetersPerSec(shooterInputs.flywheelVelocityRotationsPerSec);
    }

    // Setpoint check methods
    @AutoLogOutput(key = "Shooter/isHoodAtSetpoint")
    public boolean isHoodAtSetpoint() {
        return Math.abs(shooterInputs.hoodAngleRotations - hoodSetpointRotations) < config.hoodAngleToleranceRotations;
    }

    @AutoLogOutput(key = "Shooter/isTurretAtSetpoint")
    public boolean isTurretAtSetpoint() {
        return Math.abs(shooterInputs.turretAngleRotations - turretSetpointRotations) < config.turretAngleToleranceRotations;
    }

    @AutoLogOutput(key = "Shooter/isFlywheelAtSetpoint")
    public boolean isFlywheelAtSetpoint() {
        return Math.abs(shooterInputs.flywheelVelocityRotationsPerSec - flywheelSetpointRPS) < config.flywheelVelocityToleranceRPS;
    }

    // Config getters
    public InterpolatingMatrixTreeMap<Double, N2, N1> getLerpTable() {
        return config.getLerpTable();
    }

    public Pose3d getShooterRelativePose() {
        return config.getShooterPose3d();
    }

    public double getMinShotDistFromShooterMeters() {
        return config.minShotDistFromShooterMeters;
    }

    public double getMaxShotDistFromShooterMeters() {
        return config.maxShotDistFromShooterMeters;
    }

    public double getLatencyCompensationSeconds() {
        return config.latencyCompensationSeconds;
    }

    public double getTurretMinAngleDeg() {
        return config.turretMinAngleDeg;
    }

    public double getTurretMaxAngleDeg() {
        return config.turretMaxAngleDeg;
    }

    public double getTurretAngleToleranceDeg() {
        return config.turretAngleToleranceRotations * 360.0;
    }
}
