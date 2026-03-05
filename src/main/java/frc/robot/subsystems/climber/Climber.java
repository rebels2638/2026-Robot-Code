package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ClimberConfig;
import frc.robot.constants.Constants;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;
import frc.robot.lib.util.LoopCycleProfiler;

public class Climber extends SubsystemBase {
    private static Climber instance = null;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public enum ClimberSetpoint {
        HOME,
        READY,
        LIFTED
    }

    private final ClimberIO climberIO;
    private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    private final ClimberConfig config;

    private final DashboardMotorControlLoopConfigurator climberControlLoopConfigurator;
    private boolean pendingClimberControlLoopConfigApply = false;

    private double targetPositionRotations = 0.0;

    private Climber() {
        boolean useSimulation = Constants.shouldUseSimulation(Constants.SimOnlySubsystems.CLIMBER);
        config = ConfigLoader.load(
            "climber",
            ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.CLIMBER),
            ClimberConfig.class
        );
        climberIO = useSimulation ? new ClimberIOSim(config) : new ClimberIOTalonFX(config);

        climberControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Climber/climberControlLoop",
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.climberKP,
                config.climberKI,
                config.climberKD,
                config.climberKS,
                config.climberKV,
                config.climberKA
            )
        );
    }

    @Override
    public void periodic() {
        long periodicStartNanos = LoopCycleProfiler.markStart();

        long updateInputsStartNanos = LoopCycleProfiler.markStart();
        climberIO.updateInputs(climberInputs);
        LoopCycleProfiler.endSection("Climber/UpdateInputs", updateInputsStartNanos);

        long processInputsStartNanos = LoopCycleProfiler.markStart();
        Logger.processInputs("Climber", climberInputs);
        LoopCycleProfiler.endSection("Climber/ProcessInputs", processInputsStartNanos);

        long configUpdatesStartNanos = LoopCycleProfiler.markStart();
        pendingClimberControlLoopConfigApply |= climberControlLoopConfigurator.hasChanged();
        if (DriverStation.isDisabled() && pendingClimberControlLoopConfigApply) {
            climberIO.configureControlLoop(climberControlLoopConfigurator.getConfig());
            pendingClimberControlLoopConfigApply = false;
        }
        LoopCycleProfiler.endSection("Climber/ControlLoopConfigUpdates", configUpdatesStartNanos);

        LoopCycleProfiler.endSection("Climber/PeriodicTotal", periodicStartNanos);
    }

    private void setClimberPosition(double positionRotations) {
        targetPositionRotations = positionRotations;
        Logger.recordOutput("Climber/positionSetpointRotations", positionRotations);
        climberIO.setPosition(positionRotations);
    }

    private boolean isAtPosition(double positionRotations) {
        return Math.abs(climberInputs.positionRotations - positionRotations) < config.climberPositionToleranceRotations;
    }

    public void setSetpoint(ClimberSetpoint setpoint) {
        switch (setpoint) {
            case HOME:
                setClimberPosition(config.climberHomePositionRotations);
                break;
            case READY:
                setClimberPosition(config.climberReadyPositionRotations);
                break;
            case LIFTED:
                setClimberPosition(config.climberLiftedPositionRotations);
                break;
        }
    }

    public void setDisabled() {
        climberIO.setVoltage(0);
    }

    public void enableEStop() {
        climberIO.enableEStop();
    }

    public void disableEStop() {
        climberIO.disableEStop();
    }

    public double getClimberPositionRotations() {
        return climberInputs.positionRotations;
    }

    @AutoLogOutput(key = "Climber/isAtSetpoint")
    public boolean isAtSetpoint() {
        return isAtPosition(targetPositionRotations);
    }
}
