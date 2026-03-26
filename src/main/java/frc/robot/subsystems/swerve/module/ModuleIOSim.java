package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.configs.SwerveModuleGeneralConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;
import frc.robot.sim.MapleSimManager;

public class ModuleIOSim implements ModuleIO {
    private final MapleSimManager mapleSim = MapleSimManager.getInstance();
    private final SwerveModuleGeneralConfig config;
    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotorController;
    private final SimulatedMotorController.GenericMotorController steerMotorController;

    private PIDController driveFeedback;
    private PIDController steerFeedback;
    private SimpleMotorFeedforward driveFeedforward;

    private boolean isDriveEStopped = false;
    private boolean isSteerEStopped = false;

    public ModuleIOSim(SwerveModuleGeneralConfig config, int moduleID) {
        this.config = config;
        this.moduleSimulation = mapleSim.getModuleSimulation(moduleID);
        this.driveMotorController = moduleSimulation.useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(config.driveStatorCurrentLimit));
        this.steerMotorController = moduleSimulation.useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(config.steerStatorCurrentLimit));

        driveFeedback = new PIDController(config.driveKP, config.driveKI, config.driveKD);
        steerFeedback = new PIDController(config.steerKP, config.steerKI, config.steerKD);
        steerFeedback.enableContinuousInput(-Math.PI, Math.PI);
        driveFeedforward = new SimpleMotorFeedforward(config.driveKS, config.driveKV, config.driveKA);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        double wheelPositionRadians = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
        double wheelVelocityRadiansPerSecond = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        Rotation2d steerPosition = moduleSimulation.getSteerAbsoluteFacing();

        inputs.driveMotorConnected = true;
        inputs.steerMotorConnected = true;
        inputs.steerEncoderConnected = true;

        inputs.drivePositionMeters = wheelPositionRadians * config.driveWheelRadiusMeters;
        inputs.driveVelocityMetersPerSec = wheelVelocityRadiansPerSecond * config.driveWheelRadiusMeters;
        inputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVoltage().in(Volts);

        inputs.steerPosition = steerPosition;
        inputs.steerVelocityRadPerSec = moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.steerAppliedVolts = moduleSimulation.getSteerMotorAppliedVoltage().in(Volts);

        inputs.steerEncoderAbsolutePosition = steerPosition;
        inputs.steerEncoderPosition = steerPosition;

        inputs.driveTorqueCurrent = moduleSimulation.getDriveMotorStatorCurrent().in(Amps);
        inputs.driveTemperatureFahrenheit = 0.0;
        inputs.steerTorqueCurrent = moduleSimulation.getSteerMotorStatorCurrent().in(Amps);
        inputs.steerTemperatureFahrenheit = 0.0;

        inputs.odometryTimestampsSeconds = mapleSim.getOdometryTimestampsSeconds();
        inputs.odometryDrivePositionsMeters = convertWheelPositionCacheToMeters();
        inputs.odometrySteerPositions = moduleSimulation.getCachedSteerAbsolutePositions().clone();
    }

    private double[] convertWheelPositionCacheToMeters() {
        var cachedWheelPositions = moduleSimulation.getCachedDriveWheelFinalPositions();
        double[] drivePositionsMeters = new double[cachedWheelPositions.length];
        for (int i = 0; i < cachedWheelPositions.length; i++) {
            drivePositionsMeters[i] = cachedWheelPositions[i].in(Radians) * config.driveWheelRadiusMeters;
        }
        return drivePositionsMeters;
    }

    @Override
    public void setState(SwerveModuleState state) {
        double compensatedVelocityMetersPerSecond =
            state.speedMetersPerSecond * state.angle.minus(moduleSimulation.getSteerAbsoluteFacing()).getCos();
        requestDriveVelocity(compensatedVelocityMetersPerSecond);
        requestSteerPosition(state.angle);
    }

    @Override
    public void setSteerTorqueCurrentFOC(double torqueCurrentFOC, double driveVelocityMetersPerSec) {
        requestDriveVelocity(driveVelocityMetersPerSec);
        requestSteerVoltage(torqueCurrentFOC);
    }

    @Override
    public void setDriveTorqueCurrentFOC(double torqueCurrentFOC, Rotation2d steerAngle) {
        requestDriveVoltage(torqueCurrentFOC);
        requestSteerPosition(steerAngle);
    }

    private void requestDriveVelocity(double setpointMetersPerSecond) {
        if (isDriveEStopped) {
            requestDriveVoltage(0.0);
            return;
        }

        double measuredMetersPerSecond =
            moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond) * config.driveWheelRadiusMeters;
        double driveVolts = driveFeedforward.calculate(setpointMetersPerSecond)
            + driveFeedback.calculate(measuredMetersPerSecond, setpointMetersPerSecond);
        requestDriveVoltage(driveVolts);
    }

    private void requestSteerPosition(Rotation2d setpoint) {
        if (isSteerEStopped) {
            requestSteerVoltage(0.0);
            return;
        }

        double steerVolts = steerFeedback.calculate(
            moduleSimulation.getSteerAbsoluteFacing().getRadians(),
            setpoint.getRadians()
        );
        requestSteerVoltage(steerVolts);
    }

    private void requestDriveVoltage(double volts) {
        driveMotorController.requestVoltage(Volts.of(MathUtil.clamp(volts, -12.0, 12.0)));
    }

    private void requestSteerVoltage(double volts) {
        steerMotorController.requestVoltage(Volts.of(MathUtil.clamp(volts, -12.0, 12.0)));
    }

    @Override
    public void configureDriveControlLoop(MotorControlLoopConfig config) {
        driveFeedback = new PIDController(config.kP(), config.kI(), config.kD());
        driveFeedforward = new SimpleMotorFeedforward(config.kS(), config.kV(), config.kA());
    }

    @Override
    public void configureSteerControlLoop(MotorControlLoopConfig config) {
        steerFeedback = new PIDController(config.kP(), config.kI(), config.kD());
        steerFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void enableDriveEStop() {
        isDriveEStopped = true;
        requestDriveVoltage(0.0);
    }

    @Override
    public void disableDriveEStop() {
        isDriveEStopped = false;
    }

    @Override
    public void enableSteerEStop() {
        isSteerEStopped = true;
        requestSteerVoltage(0.0);
    }

    @Override
    public void disableSteerEStop() {
        isSteerEStopped = false;
    }
}
