package frc.robot.constants.shooter;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public abstract class ShooterConfigBase {
    // KEY: Ground distance to target (meters). VALUE: [Hood angle (rotations), Flywheel velocity (RPS)]
    public abstract InterpolatingMatrixTreeMap<Double, N2, N1> getLerpTable();

    public abstract String getCanBusName();

    // Hood motor config (positional control)
    public abstract int getHoodCanId();
    public abstract boolean getIsHoodInverted();

    public abstract double getHoodSupplyCurrentLimit();
    public abstract double getHoodSupplyCurrentLimitLowerTime();
    public abstract double getHoodSupplyCurrentLimitLowerLimit();
    public abstract double getHoodStatorCurrentLimit();
    public abstract double getHoodPeakForwardTorqueCurrent();
    public abstract double getHoodPeakReverseTorqueCurrent();

    public abstract double getHoodKS();
    public abstract double getHoodKV();
    public abstract double getHoodKA();
    public abstract double getHoodKP();
    public abstract double getHoodKI();
    public abstract double getHoodKD();

    public abstract double getHoodMotorToOutputShaftRatio();

    // Hood angle limits and starting position
    public abstract double getHoodStartingAngleRotations();
    public abstract double getHoodMinAngleRotations();
    public abstract double getHoodMaxAngleRotations();

    // Turret motor config (positional control)
    public abstract int getTurretCanId();
    public abstract boolean getIsTurretInverted();

    public abstract double getTurretSupplyCurrentLimit();
    public abstract double getTurretSupplyCurrentLimitLowerTime();
    public abstract double getTurretSupplyCurrentLimitLowerLimit();
    public abstract double getTurretStatorCurrentLimit();
    public abstract double getTurretPeakForwardTorqueCurrent();
    public abstract double getTurretPeakReverseTorqueCurrent();

    public abstract double getTurretKS();
    public abstract double getTurretKV();
    public abstract double getTurretKA();
    public abstract double getTurretKP();
    public abstract double getTurretKI();
    public abstract double getTurretKD();

    public abstract double getTurretMotorToOutputShaftRatio();

    // Turret angle limits and starting position (degrees)
    public abstract double getTurretStartingAngleDeg();
    public abstract double getTurretMinAngleDeg();
    public abstract double getTurretMaxAngleDeg();

    // Turret motion profile constraints (configured in degrees / sec and degrees / sec^2)
    public abstract double getTurretMaxVelocityDegPerSec();
    public abstract double getTurretMaxAccelerationDegPerSec2();

    // Flywheel motor config (velocity control) - 2 motors with 1:1 gearing (leader/follower)
    public abstract int getFlywheelCanId();
    public abstract int getFlywheelFollowerCanId();
    public abstract boolean getIsFlywheelInverted();
    public abstract boolean getIsFlywheelFollowerOppositeDirection();

    public abstract double getFlywheelSupplyCurrentLimit();
    public abstract double getFlywheelSupplyCurrentLimitLowerTime();
    public abstract double getFlywheelSupplyCurrentLimitLowerLimit();
    public abstract double getFlywheelStatorCurrentLimit();
    public abstract double getFlywheelPeakForwardTorqueCurrent();
    public abstract double getFlywheelPeakReverseTorqueCurrent();

    public abstract double getFlywheelKS();
    public abstract double getFlywheelKV();
    public abstract double getFlywheelKA();
    public abstract double getFlywheelKP();
    public abstract double getFlywheelKI();
    public abstract double getFlywheelKD();

    public abstract double getFlywheelMotorToOutputShaftRatio();
    public abstract double getFlywheelRadiusMeters();

    // Shooter pose relative to robot center
    public abstract Pose3d getShooterPose3d();

    // Tolerances
    public abstract double getHoodAngleToleranceRotations();
    public abstract double getTurretAngleToleranceRotations();
    public abstract double getFlywheelVelocityToleranceRPS();

    // Shot distance limits (from shooter, not robot center)
    public abstract double getMinShotDistFromShooterMeters();
    public abstract double getMaxShotDistFromShooterMeters();

    // Latency compensation for shot calculation (seconds)
    public abstract double getLatencyCompensationSeconds();
}
