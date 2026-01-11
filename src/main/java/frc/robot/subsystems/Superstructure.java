package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.VisualizeShot;
import frc.robot.constants.Constants;
import frc.robot.lib.util.ShotCalculator;
import frc.robot.lib.util.ShotCalculator.ShotData;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterCurrentState;
import frc.robot.subsystems.shooter.Shooter.ShooterDesiredState;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance;
    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    public enum CurrentState {
        STOPPED,
        HOME,
        TRACKING,
        PREPARING_FOR_SHOT,
        READY_FOR_SHOT,
        SHOOTING
    }

    public enum DesiredState {
        STOPPED,
        HOME,
        TRACKING,
        READY_FOR_SHOT,
        SHOOTING
    }

    private CurrentState currentState = CurrentState.STOPPED;
    private DesiredState desiredState = DesiredState.STOPPED;
    private CurrentState previousState = CurrentState.STOPPED;

    private final Shooter shooter = Shooter.getInstance();
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    private double lastShotTime = 0;
    private static final double SHOT_DURATION_SECONDS = 1; // Time to complete one shot

    // Margin for swerve rotation range (degrees)
    private static final double SWERVE_ROTATION_MARGIN_DEG = 20.0;
    private static final double MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC = 1.0;
    private LoggedNetworkNumber latencyCompensationSeconds = new LoggedNetworkNumber("Shooter/latencyCompSec");

    private Superstructure() {
        // Set up suppliers for the shooter - these provide dynamic setpoints based on shot calculation
        shooter.setHoodAngleSupplier(this::getTargetHoodAngle);
        shooter.setTurretAngleSupplier(this::getTargetTurretAngle);
        shooter.setFlywheelRPSSupplier(this::getTargetFlywheelRPS);
    }

    @Override
    public void periodic() {
        handleStateTransitions();
        handleCurrentState();
    }

    /**
     * Determines the next measured state based on the desired state.
     * Handles transitions between states with proper validation.
     */
    private void handleStateTransitions() {
        switch (desiredState) {
            case STOPPED:
                currentState = CurrentState.STOPPED;
                break;

            case HOME:
                if (currentState == CurrentState.SHOOTING && Timer.getTimestamp() - lastShotTime < SHOT_DURATION_SECONDS) {
                    break;
                }

                currentState = CurrentState.HOME;
                break;

            case TRACKING:
                if (currentState == CurrentState.SHOOTING && Timer.getTimestamp() - lastShotTime < SHOT_DURATION_SECONDS) {
                    break;
                }

                currentState = CurrentState.TRACKING;
                break;

            case READY_FOR_SHOT:
                if (currentState == CurrentState.SHOOTING && Timer.getTimestamp() - lastShotTime < SHOT_DURATION_SECONDS) {
                    break;
                }

                // READY_FOR_SHOT requires mechanisms to be at setpoints
                // Otherwise we're in PREPARING_FOR_SHOT
                if (isReadyForShot()) {
                    currentState = CurrentState.READY_FOR_SHOT;
                    lastShotTime = Timer.getTimestamp();
                } else {
                    currentState = CurrentState.PREPARING_FOR_SHOT;
                }
                break;

            case SHOOTING:
                // SHOOTING only allowed when we're in READY_FOR_SHOT
                if (currentState == CurrentState.READY_FOR_SHOT) {
                    currentState = CurrentState.SHOOTING;
                    new VisualizeShot();
                } else if (currentState == CurrentState.SHOOTING) {
                    // Stay in shooting, check if shot is complete
                    if (Timer.getTimestamp() - lastShotTime >= SHOT_DURATION_SECONDS) {
                        currentState = CurrentState.READY_FOR_SHOT;
                        desiredState = DesiredState.READY_FOR_SHOT;
                        lastShotTime = Timer.getTimestamp();
                    }
                } else {
                    // Not ready yet, go to preparing
                    if (isReadyForShot()) {
                        currentState = CurrentState.READY_FOR_SHOT;
                        lastShotTime = Timer.getTimestamp();
                    } else {
                        currentState = CurrentState.PREPARING_FOR_SHOT;
                    }
                }
                break;
        }

        latencyCompensationSeconds.setDefault(shooter.getLatencyCompensationSeconds());
    }

    /**
     * Executes behavior for the current state and sets subsystem states.
     */
    private void handleCurrentState() {
        switch (currentState) {
            case STOPPED:
                handleStoppedState();
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

    private void handleStoppedState() {
        shooter.setDesiredState(ShooterDesiredState.STOPPED);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.STOPPED);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.NONE);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.NONE);
    }

    private void handleHomeState() {
        shooter.setDesiredState(ShooterDesiredState.HOME);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.NONE);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.NONE);
    }

    private void handleTrackingState() {
        shooter.setDesiredState(ShooterDesiredState.TRACKING);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.NONE);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.NONE);
    }

    private void handlePreparingForShotState() {
        updateSwerveRotationRange();
        swerveDrive.setVelocityCapMaxVelocityMetersPerSec(MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC);


        shooter.setDesiredState(ShooterDesiredState.READY_FOR_SHOT);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.RANGED_ROTATION);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.CAPPED);
    }

    private void handleReadyForShotState() {
        updateSwerveRotationRange();
        swerveDrive.setVelocityCapMaxVelocityMetersPerSec(MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC);

        shooter.setDesiredState(ShooterDesiredState.READY_FOR_SHOT);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.RANGED_ROTATION);
        swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.CAPPED);
    }

    private void handleShootingState() {        
        updateSwerveRotationRange();
        swerveDrive.setVelocityCapMaxVelocityMetersPerSec(MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC);

        shooter.setDesiredState(ShooterDesiredState.SHOOTING);
        swerveDrive.setDesiredOmegaOverrideState(SwerveDrive.DesiredOmegaOverrideState.RANGED_ROTATION);

        if (Timer.getTimestamp() - lastShotTime < SHOT_DURATION_SECONDS) {
            swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.FROZEN);
        } else {
            swerveDrive.setDesiredTranslationOverrideState(SwerveDrive.DesiredTranslationOverrideState.CAPPED);
        }
    }

    /**
     * Checks if all mechanisms are at their setpoints and ready to shoot.
     * Requires swerve to be in nominal ranged rotation, shooter to be ready,
     * and robot to be within valid shooting distance.
     */
    private boolean isReadyForShot() {
        // Check if swerve is in a nominal ranged rotation state
        boolean swerveReady = 
            swerveDrive.getCurrentOmegaOverrideState() == SwerveDrive.CurrentOmegaOverrideState.RANGED_ROTATION_NOMINAL &&
            Math.hypot(
                robotState.getFieldRelativeSpeeds().vxMetersPerSecond,
                robotState.getFieldRelativeSpeeds().vyMetersPerSecond
            ) < MAX_TRANSLATIONAL_VELOCITY_DURING_SHOT_METERS_PER_SEC;
        Logger.recordOutput("Superstructure/swerveReady", swerveReady);
        // Check if shooter is ready
        boolean shooterReady = shooter.getCurrentState() == ShooterCurrentState.READY_FOR_SHOT;
        
        // Check if within valid shooting distance
        ShotData shotData = calculateShotData();
        double distance = shotData.effectiveDistance();
        boolean withinShotDistance = distance >= shooter.getMinShotDistFromShooterMeters() 
                                  && distance <= shooter.getMaxShotDistFromShooterMeters();
        
        Logger.recordOutput("Superstructure/withinShotDistance", withinShotDistance);

        return swerveReady && shooterReady && withinShotDistance;
    }

    /**
     * Updates the swerve rotation range based on turret limits and target shot angle.
     * This ensures the robot heading keeps the turret within its physical limits.
     */
    private void updateSwerveRotationRange() {
        ShotData shotData = calculateShotData();
        Logger.recordOutput("Superstructure/shotData", shotData);
        Rotation2d targetFieldYaw = shotData.targetFieldYaw();
        
        // Get turret physical limits
        double turretMinDeg = shooter.getTurretMinAngleDeg();
        double turretMaxDeg = shooter.getTurretMaxAngleDeg();
        
        // Calculate robot rotation range
        // Robot rotation = target field yaw - turret angle
        // So: turret angle = target field yaw - robot rotation
        // For turret to be within [turretMin, turretMax]:
        // Robot rotation must be within [targetYaw - turretMax, targetYaw - turretMin]
        // Add margin to ensure we stay well within bounds
        Rotation2d robotRotationMin = targetFieldYaw.minus(Rotation2d.fromDegrees(turretMaxDeg - SWERVE_ROTATION_MARGIN_DEG));
        Rotation2d robotRotationMax = targetFieldYaw.minus(Rotation2d.fromDegrees(turretMinDeg + SWERVE_ROTATION_MARGIN_DEG));
        
        swerveDrive.setRotationRange(robotRotationMin, robotRotationMax);
    }

    // Target suppliers for shooter - these calculate dynamic setpoints
    // Target suppliers always provide values from shot calculator
    // Shooter decides when to use them based on its state
    private Rotation2d getTargetHoodAngle() {
        ShotData shotData = calculateShotData();
        Logger.recordOutput("Superstructure/shotData", shotData);
        return shotData.hoodPitch();
    }

    private Rotation2d getTargetTurretAngle() {
        ShotData shotData = calculateShotData();
        Logger.recordOutput("Superstructure/shotData", shotData);
        return shotData.targetFieldYaw().minus(robotState.getEstimatedPose().getRotation());
    }

    private double getTargetFlywheelRPS() {
        ShotData shotData = calculateShotData();
        Logger.recordOutput("Superstructure/shotData", shotData);
        return shotData.flywheelRPS();
    }

    private ShotData calculateShotData() {
        Translation3d targetLocation = Constants.FieldConstants.kSHOOTER_TARGET;
        
        // Calculate shooter position in field coordinates
        Pose3d robotPose = new Pose3d(
            new Translation3d(
                robotState.getEstimatedPose().getX(), 
                robotState.getEstimatedPose().getY(), 
                0
            ),
            new Rotation3d(0, 0, robotState.getEstimatedPose().getRotation().getRadians())
        );
        Translation3d shooterPosition = robotPose.plus(new Transform3d(new Pose3d(), shooter.getShooterRelativePose())).getTranslation();
        
        ChassisSpeeds speeds = robotState.getFieldRelativeSpeeds();
        InterpolatingMatrixTreeMap<Double, N2, N1> lerpTable = shooter.getLerpTable();
        double lcomp = latencyCompensationSeconds.get();
        DoubleUnaryOperator rpsToExitVelocity = shooter::calculateShotExitVelocityMetersPerSec;
        
        // Get shooter offset from robot center (for omega compensation)
        Translation2d shooterOffsetFromRobotCenter = shooter.getShooterRelativePose().getTranslation().toTranslation2d();
        Rotation2d robotHeading = robotState.getEstimatedPose().getRotation();

        return ShotCalculator.calculate(
            targetLocation,
            shooterPosition,
            speeds,
            lerpTable,
            lcomp,
            rpsToExitVelocity,
            shooterOffsetFromRobotCenter,
            robotHeading
        );
    }

    // Public interface
    public void setDesiredState(DesiredState desiredState) {
        this.desiredState = desiredState;
    }

    @AutoLogOutput(key = "Superstructure/currentState")
    public CurrentState getCurrentState() {
        return currentState;
    }

    @AutoLogOutput(key = "Superstructure/desiredState")
    public DesiredState getDesiredState() {
        return desiredState;
    }
}
