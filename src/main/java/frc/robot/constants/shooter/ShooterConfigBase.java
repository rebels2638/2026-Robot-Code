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

    // Flywheel motor config (velocity control)
    public abstract int getFlywheelCanId();
    public abstract boolean getIsFlywheelInverted();

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

    // Feeder motor config (velocity control)
    public abstract int getFeederCanId();
    public abstract boolean getIsFeederInverted();

    public abstract double getFeederSupplyCurrentLimit();
    public abstract double getFeederSupplyCurrentLimitLowerTime();
    public abstract double getFeederSupplyCurrentLimitLowerLimit();
    public abstract double getFeederStatorCurrentLimit();
    public abstract double getFeederPeakForwardTorqueCurrent();
    public abstract double getFeederPeakReverseTorqueCurrent();

    public abstract double getFeederKS();
    public abstract double getFeederKV();
    public abstract double getFeederKA();
    public abstract double getFeederKP();
    public abstract double getFeederKI();
    public abstract double getFeederKD();

    public abstract double getFeederMotorToOutputShaftRatio();

    // Indexer motor config (velocity control)
    public abstract int getIndexerCanId();
    public abstract boolean getIsIndexerInverted();

    public abstract double getIndexerSupplyCurrentLimit();
    public abstract double getIndexerSupplyCurrentLimitLowerTime();
    public abstract double getIndexerSupplyCurrentLimitLowerLimit();
    public abstract double getIndexerStatorCurrentLimit();
    public abstract double getIndexerPeakForwardTorqueCurrent();
    public abstract double getIndexerPeakReverseTorqueCurrent();

    public abstract double getIndexerKS();
    public abstract double getIndexerKV();
    public abstract double getIndexerKA();
    public abstract double getIndexerKP();
    public abstract double getIndexerKI();
    public abstract double getIndexerKD();

    public abstract double getIndexerMotorToOutputShaftRatio();

    // Tolerances
    public abstract double getHoodAngleToleranceRotations();
    public abstract double getTurretAngleToleranceRotations();
    public abstract double getFlywheelVelocityToleranceRPS();
    public abstract double getFeederVelocityToleranceRPS();
    public abstract double getIndexerVelocityToleranceRPS();

    // Shot distance limits (from shooter, not robot center)
    public abstract double getMinShotDistFromShooterMeters();
    public abstract double getMaxShotDistFromShooterMeters();

    // Latency compensation for shot calculation (seconds)
    public abstract double getLatencyCompensationSeconds();
}
