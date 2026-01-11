package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.constants.shooter.ShooterConfigBase;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;
import frc.robot.lib.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX hoodMotor;
    private final TalonFX turretMotor;
    private final TalonFX flywheelMotor;
    private final TalonFX feederMotor;
    private final TalonFX indexerMotor;

    private final StatusSignal<Angle> hoodPositionStatusSignal;
    private final StatusSignal<AngularVelocity> hoodVelocityStatusSignal;

    private final StatusSignal<Angle> turretPositionStatusSignal;
    private final StatusSignal<AngularVelocity> turretVelocityStatusSignal;

    private final StatusSignal<AngularVelocity> flywheelVelocityStatusSignal;
    private final StatusSignal<AngularVelocity> feederVelocityStatusSignal;
    private final StatusSignal<AngularVelocity> indexerVelocityStatusSignal;

    private final StatusSignal<Current> hoodTorqueCurrent;
    private final StatusSignal<Temperature> hoodTemperature;

    private final StatusSignal<Current> turretTorqueCurrent;
    private final StatusSignal<Temperature> turretTemperature;

    private final StatusSignal<Current> flywheelTorqueCurrent;
    private final StatusSignal<Temperature> flywheelTemperature;

    private final StatusSignal<Current> feederTorqueCurrent;
    private final StatusSignal<Temperature> feederTemperature;

    private final StatusSignal<Current> indexerTorqueCurrent;
    private final StatusSignal<Temperature> indexerTemperature;

    // private final MotionMagicTorqueCurrentFOC hoodMotorRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
    private final PositionVoltage hoodMotorRequest = new PositionVoltage(0).withSlot(0);
    private final MotionMagicVoltage turretMotorRequest = new MotionMagicVoltage(0).withSlot(0);

    private final VelocityVoltage flywheelMotorRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage feederMotorRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage indexerMotorRequest = new VelocityVoltage(0).withSlot(0);

    private final ShooterConfigBase config;
    private final TalonFXConfiguration hoodConfig;
    private final TalonFXConfiguration turretConfig;
    private final TalonFXConfiguration flywheelConfig;
    private final TalonFXConfiguration feederConfig;
    private final TalonFXConfiguration indexerConfig;

    public ShooterIOTalonFX(ShooterConfigBase config) {
        this.config = config;

        // Hood motor configuration (positional control)
        hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = config.getHoodKP();
        hoodConfig.Slot0.kI = config.getHoodKI();
        hoodConfig.Slot0.kD = config.getHoodKD();
        hoodConfig.Slot0.kS = config.getHoodKS();
        hoodConfig.Slot0.kV = config.getHoodKV();
        hoodConfig.Slot0.kA = config.getHoodKA();
        hoodConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        hoodConfig.ClosedLoopGeneral.ContinuousWrap = false;
        hoodConfig.Feedback.SensorToMechanismRatio = config.getHoodMotorToOutputShaftRatio();

        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = config.getIsHoodInverted() ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = config.getHoodSupplyCurrentLimit();
        hoodConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getHoodSupplyCurrentLimitLowerLimit();
        hoodConfig.CurrentLimits.SupplyCurrentLowerTime = config.getHoodSupplyCurrentLimitLowerTime();

        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = config.getHoodStatorCurrentLimit();

        hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getHoodPeakForwardTorqueCurrent();
        hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getHoodPeakReverseTorqueCurrent();

        // Software limits for hood angle
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.getHoodMaxAngleRotations();
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.getHoodMinAngleRotations();

        hoodConfig.FutureProofConfigs = false;

        hoodMotor = new TalonFX(config.getHoodCanId(), config.getCanBusName());
        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.setPosition(config.getHoodStartingAngleRotations(), 0.25));

        // Turret motor configuration (positional control, mirrors hood, Motion Magic)
        turretConfig = new TalonFXConfiguration();
        turretConfig.Slot0.kP = config.getTurretKP();
        turretConfig.Slot0.kI = config.getTurretKI();
        turretConfig.Slot0.kD = config.getTurretKD();
        turretConfig.Slot0.kS = config.getTurretKS();
        turretConfig.Slot0.kV = config.getTurretKV();
        turretConfig.Slot0.kA = config.getTurretKA();
        turretConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        turretConfig.ClosedLoopGeneral.ContinuousWrap = false;
        turretConfig.Feedback.SensorToMechanismRatio = config.getTurretMotorToOutputShaftRatio();

        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretConfig.MotorOutput.Inverted = config.getIsTurretInverted() ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretConfig.CurrentLimits.SupplyCurrentLimit = config.getTurretSupplyCurrentLimit();
        turretConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getTurretSupplyCurrentLimitLowerLimit();
        turretConfig.CurrentLimits.SupplyCurrentLowerTime = config.getTurretSupplyCurrentLimitLowerTime();

        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretConfig.CurrentLimits.StatorCurrentLimit = config.getTurretStatorCurrentLimit();

        turretConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getTurretPeakForwardTorqueCurrent();
        turretConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getTurretPeakReverseTorqueCurrent();

        // Software limits for turret angle (convert degrees to rotations)
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.getTurretMaxAngleDeg() / 360.0;
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.getTurretMinAngleDeg() / 360.0;

        // Motion Magic constraints (degrees/sec -> rotations/sec)
        double turretCruiseVelocityRotPerSec = config.getTurretMaxVelocityDegPerSec() / 360.0;
        double turretAccelerationRotPerSec2 = config.getTurretMaxAccelerationDegPerSec2() / 360.0;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = turretCruiseVelocityRotPerSec;
        turretConfig.MotionMagic.MotionMagicAcceleration = turretAccelerationRotPerSec2;

        turretConfig.FutureProofConfigs = false;

        turretMotor = new TalonFX(config.getTurretCanId(), config.getCanBusName());
        PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(turretConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> turretMotor.setPosition(config.getTurretStartingAngleDeg() / 360.0, 0.25));

        // Flywheel motor configuration (velocity control)
        flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kP = config.getFlywheelKP();
        flywheelConfig.Slot0.kI = config.getFlywheelKI();
        flywheelConfig.Slot0.kD = config.getFlywheelKD();
        flywheelConfig.Slot0.kS = config.getFlywheelKS();
        flywheelConfig.Slot0.kV = config.getFlywheelKV();
        flywheelConfig.Slot0.kA = config.getFlywheelKA();
        flywheelConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        flywheelConfig.ClosedLoopGeneral.ContinuousWrap = false;
        flywheelConfig.Feedback.SensorToMechanismRatio = config.getFlywheelMotorToOutputShaftRatio();

        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.MotorOutput.Inverted = config.getIsFlywheelInverted() ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = config.getFlywheelSupplyCurrentLimit();
        flywheelConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getFlywheelSupplyCurrentLimitLowerLimit();
        flywheelConfig.CurrentLimits.SupplyCurrentLowerTime = config.getFlywheelSupplyCurrentLimitLowerTime();

        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = config.getFlywheelStatorCurrentLimit();

        flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getFlywheelPeakForwardTorqueCurrent();
        flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getFlywheelPeakReverseTorqueCurrent();

        flywheelConfig.FutureProofConfigs = false;

        flywheelMotor = new TalonFX(config.getFlywheelCanId(), config.getCanBusName());
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));

        // Feeder motor configuration (velocity control)
        feederConfig = new TalonFXConfiguration();

        feederConfig.Slot0.kP = config.getFeederKP();
        feederConfig.Slot0.kI = config.getFeederKI();
        feederConfig.Slot0.kD = config.getFeederKD();
        feederConfig.Slot0.kS = config.getFeederKS();
        feederConfig.Slot0.kV = config.getFeederKV();
        feederConfig.Slot0.kA = config.getFeederKA();
        feederConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        feederConfig.ClosedLoopGeneral.ContinuousWrap = false;
        feederConfig.Feedback.SensorToMechanismRatio = config.getFeederMotorToOutputShaftRatio();

        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.MotorOutput.Inverted = config.getIsFeederInverted() ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederConfig.CurrentLimits.SupplyCurrentLimit = config.getFeederSupplyCurrentLimit();
        feederConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getFeederSupplyCurrentLimitLowerLimit();
        feederConfig.CurrentLimits.SupplyCurrentLowerTime = config.getFeederSupplyCurrentLimitLowerTime();

        feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfig.CurrentLimits.StatorCurrentLimit = config.getFeederStatorCurrentLimit();

        feederConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getFeederPeakForwardTorqueCurrent();
        feederConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getFeederPeakReverseTorqueCurrent();

        feederConfig.FutureProofConfigs = false;

        feederMotor = new TalonFX(config.getFeederCanId(), config.getCanBusName());
        PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(feederConfig, 0.25));

        // Indexer motor configuration (velocity control)
        indexerConfig = new TalonFXConfiguration();

        indexerConfig.Slot0.kP = config.getIndexerKP();
        indexerConfig.Slot0.kI = config.getIndexerKI();
        indexerConfig.Slot0.kD = config.getIndexerKD();
        indexerConfig.Slot0.kS = config.getIndexerKS();
        indexerConfig.Slot0.kV = config.getIndexerKV();
        indexerConfig.Slot0.kA = config.getIndexerKA();
        indexerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        indexerConfig.ClosedLoopGeneral.ContinuousWrap = false;
        indexerConfig.Feedback.SensorToMechanismRatio = config.getIndexerMotorToOutputShaftRatio();

        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        indexerConfig.MotorOutput.Inverted = config.getIsIndexerInverted() ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = config.getIndexerSupplyCurrentLimit();
        indexerConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getIndexerSupplyCurrentLimitLowerLimit();
        indexerConfig.CurrentLimits.SupplyCurrentLowerTime = config.getIndexerSupplyCurrentLimitLowerTime();

        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfig.CurrentLimits.StatorCurrentLimit = config.getIndexerStatorCurrentLimit();

        indexerConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getIndexerPeakForwardTorqueCurrent();
        indexerConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getIndexerPeakReverseTorqueCurrent();

        indexerConfig.FutureProofConfigs = false;

        indexerMotor = new TalonFX(config.getIndexerCanId(), config.getCanBusName());
        PhoenixUtil.tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(indexerConfig, 0.25));


        // Status signals
        hoodTorqueCurrent = hoodMotor.getTorqueCurrent().clone();
        hoodTemperature = hoodMotor.getDeviceTemp().clone();

        turretTorqueCurrent = turretMotor.getTorqueCurrent().clone();
        turretTemperature = turretMotor.getDeviceTemp().clone();

        flywheelTorqueCurrent = flywheelMotor.getTorqueCurrent().clone();
        flywheelTemperature = flywheelMotor.getDeviceTemp().clone();

        feederTorqueCurrent = feederMotor.getTorqueCurrent().clone();
        feederTemperature = feederMotor.getDeviceTemp().clone();

        indexerTorqueCurrent = indexerMotor.getTorqueCurrent().clone();
        indexerTemperature = indexerMotor.getDeviceTemp().clone();

        hoodPositionStatusSignal = hoodMotor.getPosition().clone();
        hoodVelocityStatusSignal = hoodMotor.getVelocity().clone();

        turretPositionStatusSignal = turretMotor.getPosition().clone();
        turretVelocityStatusSignal = turretMotor.getVelocity().clone();

        flywheelVelocityStatusSignal = flywheelMotor.getVelocity().clone();
        feederVelocityStatusSignal = feederMotor.getVelocity().clone();
        indexerVelocityStatusSignal = indexerMotor.getVelocity().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(100,
            hoodTorqueCurrent, hoodTemperature,
            turretTorqueCurrent, turretTemperature,
            flywheelTorqueCurrent, flywheelTemperature,
            feederTorqueCurrent, feederTemperature,
            indexerTorqueCurrent, indexerTemperature,
            hoodPositionStatusSignal, hoodVelocityStatusSignal,
            turretPositionStatusSignal, turretVelocityStatusSignal,
            flywheelVelocityStatusSignal, feederVelocityStatusSignal,
            indexerVelocityStatusSignal);

        hoodMotor.optimizeBusUtilization();
        turretMotor.optimizeBusUtilization();
        flywheelMotor.optimizeBusUtilization();
        feederMotor.optimizeBusUtilization();
        indexerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            hoodTorqueCurrent, hoodTemperature,
            turretTorqueCurrent, turretTemperature,
            flywheelTorqueCurrent, flywheelTemperature,
            feederTorqueCurrent, feederTemperature,
            indexerTorqueCurrent, indexerTemperature,
            hoodPositionStatusSignal, hoodVelocityStatusSignal,
            turretPositionStatusSignal, turretVelocityStatusSignal,
            flywheelVelocityStatusSignal, feederVelocityStatusSignal,
            indexerVelocityStatusSignal);

        inputs.hoodAngleRotations = hoodPositionStatusSignal.getValue().in(Rotations);
        inputs.hoodVelocityRotationsPerSec = hoodVelocityStatusSignal.getValue().in(RotationsPerSecond);

        inputs.turretAngleRotations = turretPositionStatusSignal.getValue().in(Rotations);
        inputs.turretVelocityRotationsPerSec = turretVelocityStatusSignal.getValue().in(RotationsPerSecond);

        inputs.flywheelVelocityRotationsPerSec = flywheelVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.flywheelAppliedVolts = flywheelMotor.getMotorVoltage().getValueAsDouble();

        inputs.feederVelocityRotationsPerSec = feederVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.feederAppliedVolts = feederMotor.getMotorVoltage().getValueAsDouble();
        inputs.flywheelTorqueCurrent = flywheelTorqueCurrent.getValue().in(Amps);

        inputs.indexerVelocityRotationsPerSec = indexerVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.indexerAppliedVolts = indexerMotor.getMotorVoltage().getValueAsDouble();

        inputs.hoodTorqueCurrent = hoodTorqueCurrent.getValue().in(Amps);
        inputs.hoodTemperatureFahrenheit = hoodTemperature.getValue().in(Fahrenheit);

        inputs.turretTorqueCurrent = turretTorqueCurrent.getValue().in(Amps);
        inputs.turretTemperatureFahrenheit = turretTemperature.getValue().in(Fahrenheit);

        inputs.flywheelTemperatureFahrenheit = flywheelTemperature.getValue().in(Fahrenheit);

        inputs.feederTemperatureFahrenheit = feederTemperature.getValue().in(Fahrenheit);

        inputs.indexerTemperatureFahrenheit = indexerTemperature.getValue().in(Fahrenheit);

        Logger.recordOutput("Shooter/turretTalonSetpoint", turretMotor.getClosedLoopReference().getValueAsDouble());
    }

    @Override
    public void setAngle(double angleRotations) {
        // Clamp angle within software limits
        double clampedAngle = Math.max(config.getHoodMinAngleRotations(),
            Math.min(config.getHoodMaxAngleRotations(), angleRotations));
        hoodMotor.setControl(hoodMotorRequest.withPosition(clampedAngle));
    }

    @Override
    public void setTurretAngle(double angleRotations) {
        // Clamp angle within software limits
        double minRot = config.getTurretMinAngleDeg() / 360.0;
        double maxRot = config.getTurretMaxAngleDeg() / 360.0;
        double clampedAngle = Math.max(minRot, Math.min(maxRot, angleRotations));
        turretMotor.setControl(turretMotorRequest.withPosition(clampedAngle));
    }

    @Override
    public void setShotVelocity(double velocityRotationsPerSec) {
        flywheelMotor.setControl(flywheelMotorRequest.withVelocity(velocityRotationsPerSec));
    }

    @Override
    public void setFeedVelocity(double velocityRotationsPerSec) {
        feederMotor.setControl(feederMotorRequest.withVelocity(velocityRotationsPerSec));
    }

    @Override
    public void setHoodTorqueCurrentFOC(double torqueCurrentFOC) {
        hoodMotor.setControl(new TorqueCurrentFOC(torqueCurrentFOC));
    }

    @Override
    public void setTurretTorqueCurrentFOC(double torqueCurrentFOC) {
        turretMotor.setControl(new TorqueCurrentFOC(torqueCurrentFOC));
    }

    @Override
    public void setFlywheelVoltage(double voltage) {
        flywheelMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setFeederVoltage(double voltage) {
        feederMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setIndexerVelocity(double velocityRotationsPerSec) {
        indexerMotor.setControl(indexerMotorRequest.withVelocity(velocityRotationsPerSec));
    }

    @Override
    public void setIndexerVoltage(double voltage) {
        indexerMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void configureHoodControlLoop(MotorControlLoopConfig config) {
        hoodConfig.Slot0.kP = config.kP();
        hoodConfig.Slot0.kI = config.kI();
        hoodConfig.Slot0.kD = config.kD();
        hoodConfig.Slot0.kS = config.kS();
        hoodConfig.Slot0.kV = config.kV();
        hoodConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
    }

    @Override
    public void configureTurretControlLoop(MotorControlLoopConfig config) {
        turretConfig.Slot0.kP = config.kP();
        turretConfig.Slot0.kI = config.kI();
        turretConfig.Slot0.kD = config.kD();
        turretConfig.Slot0.kS = config.kS();
        turretConfig.Slot0.kV = config.kV();
        turretConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(turretConfig, 0.25));
    }

    @Override
    public void configureFlywheelControlLoop(MotorControlLoopConfig config) {
        flywheelConfig.Slot0.kP = config.kP();
        flywheelConfig.Slot0.kI = config.kI();
        flywheelConfig.Slot0.kD = config.kD();
        flywheelConfig.Slot0.kS = config.kS();
        flywheelConfig.Slot0.kV = config.kV();
        flywheelConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));
    }

    @Override
    public void configureFeederControlLoop(MotorControlLoopConfig config) {
        feederConfig.Slot0.kP = config.kP();
        feederConfig.Slot0.kI = config.kI();
        feederConfig.Slot0.kD = config.kD();
        feederConfig.Slot0.kS = config.kS();
        feederConfig.Slot0.kV = config.kV();
        feederConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(feederConfig, 0.25));
    }

    @Override
    public void configureIndexerControlLoop(MotorControlLoopConfig config) {
        indexerConfig.Slot0.kP = config.kP();
        indexerConfig.Slot0.kI = config.kI();
        indexerConfig.Slot0.kD = config.kD();
        indexerConfig.Slot0.kS = config.kS();
        indexerConfig.Slot0.kV = config.kV();
        indexerConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(indexerConfig, 0.25));
    }

}
