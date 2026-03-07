package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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

    public enum DesiredState {
        DISABLED,
        RETRACTED,
        EXTENDED,
        CLIMBING
    }

    public enum CurrentState {
        DISABLED,
        HOME,
        RETRACTING,
        EXTENDING,
        EXTENDED,
        CLIMBING
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

    private DesiredState desiredState = DesiredState.DISABLED;
    private CurrentState currentState = CurrentState.DISABLED;
    private boolean pendingClimberControlLoopConfigApply = false;
    private double targetPositionRotations = 0.0;
    private final boolean enableConnectionAlerts;
    private final Alert climberDisconnectedAlert;

    private Climber() {
        boolean useSimulation = Constants.shouldUseSimulation(Constants.SimOnlySubsystems.CLIMBER);
        config = ConfigLoader.load(
            "climber",
            ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.CLIMBER),
            ClimberConfig.class
        );
        climberIO = useSimulation ? new ClimberIOSim(config) : new ClimberIOTalonFX(config);
        enableConnectionAlerts = !useSimulation && Constants.currentMode != Constants.Mode.REPLAY;
        climberDisconnectedAlert = new Alert("Climber motor is disconnected.", AlertType.kWarning);

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

        climberDisconnectedAlert.set(enableConnectionAlerts && !climberInputs.climberMotorConnected);

        long stateTransitionStartNanos = LoopCycleProfiler.markStart();
        handleStateTransitions();
        LoopCycleProfiler.endSection("Climber/StateTransitions", stateTransitionStartNanos);

        long currentStateStartNanos = LoopCycleProfiler.markStart();
        handleCurrentState();
        LoopCycleProfiler.endSection("Climber/CurrentStateHandling", currentStateStartNanos);

        long configUpdatesStartNanos = LoopCycleProfiler.markStart();
        pendingClimberControlLoopConfigApply |= climberControlLoopConfigurator.hasChanged();
        if (DriverStation.isDisabled() && pendingClimberControlLoopConfigApply) {
            climberIO.configureControlLoop(climberControlLoopConfigurator.getConfig());
            pendingClimberControlLoopConfigApply = false;
        }
        LoopCycleProfiler.endSection("Climber/ControlLoopConfigUpdates", configUpdatesStartNanos);

        LoopCycleProfiler.endSection("Climber/PeriodicTotal", periodicStartNanos);
    }

    private void handleStateTransitions() {
        switch (desiredState) {
            case DISABLED:
                currentState = CurrentState.DISABLED;
                break;
            case RETRACTED:
                currentState = resolveMotionState(config.climberHomePositionRotations, CurrentState.HOME);
                break;
            case EXTENDED:
                currentState = resolveMotionState(config.climberReadyPositionRotations, CurrentState.EXTENDED);
                break;
            case CLIMBING:
                currentState = resolveMotionState(config.climberLiftedPositionRotations, CurrentState.CLIMBING);
                break;
        }
    }

    private CurrentState resolveMotionState(double goalPositionRotations, CurrentState settledState) {
        if (isAtPosition(goalPositionRotations)) {
            return settledState;
        }
        return goalPositionRotations > climberInputs.positionRotations
            ? CurrentState.EXTENDING
            : CurrentState.RETRACTING;
    }

    private void handleCurrentState() {
        switch (currentState) {
            case DISABLED:
                setDisabled();
                break;
            case HOME:
            case RETRACTING:
                setClimberPosition(config.climberHomePositionRotations);
                break;
            case EXTENDING:
            case EXTENDED:
                setClimberPosition(config.climberReadyPositionRotations);
                break;
            case CLIMBING:
                setClimberPosition(config.climberLiftedPositionRotations);
                break;
        }
    }

    private void setClimberPosition(double positionRotations) {
        targetPositionRotations = positionRotations;
        Logger.recordOutput("Climber/positionSetpointRotations", positionRotations);
        climberIO.setPosition(positionRotations);
    }

    private boolean isAtPosition(double positionRotations) {
        return Math.abs(climberInputs.positionRotations - positionRotations) < config.climberPositionToleranceRotations;
    }

    public void setDesiredState(DesiredState desiredState) {
        this.desiredState = desiredState;
    }

    public void setSetpoint(ClimberSetpoint setpoint) {
        switch (setpoint) {
            case HOME:
                setDesiredState(DesiredState.RETRACTED);
                break;
            case READY:
                setDesiredState(DesiredState.EXTENDED);
                break;
            case LIFTED:
                setDesiredState(DesiredState.CLIMBING);
                break;
        }
    }

    public void setDisabled() {
        targetPositionRotations = climberInputs.positionRotations;
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

    @AutoLogOutput(key = "Climber/desiredState")
    public DesiredState getDesiredState() {
        return desiredState;
    }

    @AutoLogOutput(key = "Climber/currentState")
    public CurrentState getCurrentState() {
        return currentState;
    }

    @AutoLogOutput(key = "Climber/isAtSetpoint")
    public boolean isAtSetpoint() {
        return climberInputs.climberMotorConnected && isAtPosition(targetPositionRotations);
    }

    @AutoLogOutput(key = "Climber/isAtDesiredState")
    public boolean isAtDesiredState() {
        return switch (desiredState) {
            case DISABLED -> currentState == CurrentState.DISABLED;
            case RETRACTED -> currentState == CurrentState.HOME;
            case EXTENDED -> currentState == CurrentState.EXTENDED;
            case CLIMBING -> currentState == CurrentState.CLIMBING;
        };
    }

    @AutoLogOutput(key = "Climber/isHome")
    public boolean isHome() {
        return climberInputs.climberMotorConnected && currentState == CurrentState.HOME;
    }

    @AutoLogOutput(key = "Climber/isExtended")
    public boolean isExtended() {
        return climberInputs.climberMotorConnected && currentState == CurrentState.EXTENDED;
    }

    @AutoLogOutput(key = "Climber/isClimberMotorConnected")
    public boolean isClimberMotorConnected() {
        return climberInputs.climberMotorConnected;
    }
}
