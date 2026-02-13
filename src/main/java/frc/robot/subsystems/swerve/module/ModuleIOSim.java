package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.configs.SwerveModuleGeneralConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Module IO implementation backed by maple-sim's SwerveModuleSimulation.
 * Each instance wraps one of the four SwerveModuleSimulation objects
 * obtained from the SwerveDriveSimulation.
 */
public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotorController;
    private final SimulatedMotorController.GenericMotorController steerMotorController;
    private final int moduleID;

    private PIDController driveFeedback;
    private PIDController steerFeedback;
    private SimpleMotorFeedforward driveFeedforward;

    private boolean isDriveClosedLoop = false;
    private boolean isSteerClosedLoop = false;
    private boolean isDriveEStopped = false;
    private boolean isSteerEStopped = false;

    private SwerveModuleState lastDesiredState = new SwerveModuleState();

    private final double wheelRadiusMeters;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation, SwerveModuleGeneralConfig config, int moduleID) {
        this.moduleSimulation = moduleSimulation;
        this.moduleID = moduleID;
        this.wheelRadiusMeters = config.driveWheelRadiusMeters;

        this.driveMotorController = moduleSimulation.useGenericMotorControllerForDrive();
        this.steerMotorController = moduleSimulation.useGenericControllerForSteer();

        driveFeedback = new PIDController(0.05, 0.0, 0.0);
        steerFeedback = new PIDController(8.0, 0.0, 0.0);
        steerFeedback.enableContinuousInput(-Math.PI, Math.PI);

        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if (isDriveEStopped) {
            driveMotorController.requestVoltage(Volts.of(0));
        } else if (isDriveClosedLoop) {
            double driveVelocityMPS = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond)
                    * wheelRadiusMeters;
            double driveVolts = driveFeedforward.calculate(lastDesiredState.speedMetersPerSecond)
                    + driveFeedback.calculate(driveVelocityMPS, lastDesiredState.speedMetersPerSecond);
            driveMotorController.requestVoltage(Volts.of(MathUtil.clamp(driveVolts, -12, 12)));
        }

        if (isSteerEStopped) {
            steerMotorController.requestVoltage(Volts.of(0));
        } else if (isSteerClosedLoop) {
            double steerVolts = steerFeedback.calculate(
                    moduleSimulation.getSteerAbsoluteFacing().getRadians());
            steerMotorController.requestVoltage(Volts.of(MathUtil.clamp(steerVolts, -12, 12)));
        }

        inputs.drivePositionMeters = moduleSimulation.getDriveWheelFinalPosition().in(Radians)
                * wheelRadiusMeters;
        inputs.driveVelocityMetersPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond)
                * wheelRadiusMeters;
        inputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVoltage().in(Volts);

        inputs.steerPosition = moduleSimulation.getSteerAbsoluteFacing();
        inputs.steerVelocityRadPerSec = moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.steerAppliedVolts = moduleSimulation.getSteerMotorAppliedVoltage().in(Volts);

        inputs.steerEncoderAbsolutePosition = inputs.steerPosition;
        inputs.steerEncoderPosition = inputs.steerPosition;

        inputs.driveTorqueCurrent = moduleSimulation.getDriveMotorSupplyCurrent().in(Amps);
        inputs.steerTorqueCurrent = moduleSimulation.getSteerMotorSupplyCurrent().in(Amps);

        Angle[] cachedDrivePositions = moduleSimulation.getCachedDriveWheelFinalPositions();

        double currentTime = Timer.getFPGATimestamp();
        double dt = 0.02 / cachedDrivePositions.length;

        inputs.odometryTimestampsSeconds = new double[cachedDrivePositions.length];
        for (int i = 0; i < cachedDrivePositions.length; i++) {
            inputs.odometryTimestampsSeconds[i] = currentTime - (cachedDrivePositions.length - 1 - i) * dt;
        }

        inputs.odometryDrivePositionsMeters = new double[cachedDrivePositions.length];
        for (int i = 0; i < cachedDrivePositions.length; i++) {
            inputs.odometryDrivePositionsMeters[i] = cachedDrivePositions[i].in(Radians) * wheelRadiusMeters;
        }

        Rotation2d[] cachedSteerPositions = moduleSimulation.getCachedSteerAbsolutePositions();
        inputs.odometrySteerPositions = cachedSteerPositions;
    }

    @Override
    public void setState(SwerveModuleState state) {
        lastDesiredState = state;

        if (isDriveEStopped) {
            isDriveClosedLoop = false;
        } else {
            driveFeedback.setSetpoint(state.speedMetersPerSecond);
            isDriveClosedLoop = true;
        }

        if (isSteerEStopped) {
            isSteerClosedLoop = false;
        } else {
            steerFeedback.setSetpoint(state.angle.getRadians());
            isSteerClosedLoop = true;
        }
    }

    @Override
    public void setSteerTorqueCurrentFOC(double torqueCurrentFOC, double driveVelocityMetersPerSec) {
        steerMotorController.requestVoltage(Volts.of(isSteerEStopped ? 0 : torqueCurrentFOC));
        isDriveClosedLoop = false;
        isSteerClosedLoop = false;
    }

    @Override
    public void setDriveTorqueCurrentFOC(double torqueCurrentFOC, Rotation2d steerAngle) {
        driveMotorController.requestVoltage(Volts.of(isDriveEStopped ? 0 : torqueCurrentFOC));
        isDriveClosedLoop = false;
        isSteerClosedLoop = true;
        steerFeedback.setSetpoint(steerAngle.getRadians());
    }

    @Override
    public void configureDriveControlLoop(MotorControlLoopConfig config) {
        // No-op in sim — hardware-level PID tuning not applicable
    }

    @Override
    public void configureSteerControlLoop(MotorControlLoopConfig config) {
        // No-op in sim
    }

    @Override
    public void enableDriveEStop() {
        isDriveEStopped = true;
        driveMotorController.requestVoltage(Volts.of(0));
        isDriveClosedLoop = false;
    }

    @Override
    public void disableDriveEStop() {
        isDriveEStopped = false;
    }

    @Override
    public void enableSteerEStop() {
        isSteerEStopped = true;
        steerMotorController.requestVoltage(Volts.of(0));
        isSteerClosedLoop = false;
    }

    @Override
    public void disableSteerEStop() {
        isSteerEStopped = false;
    }
}
