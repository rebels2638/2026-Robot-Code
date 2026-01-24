package frc.robot.configs;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModuleGeneralConfig {
    public String canBusName;

    public double driveSupplyCurrentLimit;
    public double driveSupplyCurrentLimitLowerTime;
    public double driveSupplyCurrentLimitLowerLimit;
    public double driveStatorCurrentLimit;
    public double drivePeakForwardTorqueCurrent;
    public double drivePeakReverseTorqueCurrent;

    public double driveKS;
    public double driveKV;
    public double driveKA;
    public double driveKP;
    public double driveKI;
    public double driveKD;

    public double driveMotionMagicVelocityAccelerationMetersPerSecSec;
    public double driveMotionMagicVelocityDecelerationMetersPerSecSec;
    public double driveMotionMagicVelocityJerkMetersPerSecSecSec;

    public double driveMaxVelocityMetersPerSec;
    public boolean isDriveNeutralModeBrake;

    public double driveMotorToOutputShaftRatio;
    public double driveWheelRadiusMeters;

    public double steerSupplyCurrentLimit;
    public double steerSupplyCurrentLimitLowerTime;
    public double steerSupplyCurrentLimitLowerLimit;
    public double steerStatorCurrentLimit;
    public double steerPeakForwardTorqueCurrent;
    public double steerPeakReverseTorqueCurrent;

    public double steerKS;
    public double steerKV;
    public double steerKA;
    public double steerKP;
    public double steerKI;
    public double steerKD;

    public double steerMotionMagicExpoKA;
    public double steerMotionMagicExpoKV;
    public double steerMotionMagicCruiseVelocityRotationsPerSec;

    public boolean isSteerNeutralModeBrake;

    public double steerMotorToOutputShaftRatio;
    public double steerRotorToSensorRatio;

    public String steerCancoderFeedbackSensorSource;
    public String cancoderSensorDirection;
    public double cancoderAbsoluteSensorDiscontinuityPoint;

    public double driveMinWallCurrent;
    public double driveMaxWallVeloMetersPerSec;

    public FeedbackSensorSourceValue getSteerCancoderFeedbackSensorSource() {
        return FeedbackSensorSourceValue.valueOf(steerCancoderFeedbackSensorSource);
    }

    public SensorDirectionValue getCancoderSensorDirection() {
        return SensorDirectionValue.valueOf(cancoderSensorDirection);
    }
}
