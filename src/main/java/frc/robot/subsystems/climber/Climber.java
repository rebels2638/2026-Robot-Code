package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ClimberConfig;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Climber extends SubsystemBase {
    private static Climber instance = null;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public enum ClimberCurrentState {
        STOPPED,
        HOME,
        TRANSITIONING_TO_HOME,
        READY,
        TRANSITIONING_TO_READY,
        LIFTED,
        TRANSITIONING_TO_LIFTED
    }

    public enum ClimberDesiredState {
        STOPPED,
        HOME,
        READY,
        LIFTED
    }

    private ClimberCurrentState currentState = ClimberCurrentState.STOPPED;
    private ClimberDesiredState desiredState = ClimberDesiredState.STOPPED;

    private final ClimberIO climberIO;
    private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    private final ClimberConfig config;

    private final DashboardMotorControlLoopConfigurator climberControlLoopConfigurator;

    private double targetPositionRotations = 0.0;

    private Climber() {
        config = ConfigLoader.load("climber", ClimberConfig.class);
        climberIO = RobotBase.isSimulation() ? new ClimberIOSim(config) : new ClimberIOTalonFX(config);

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
        climberIO.updateInputs(climberInputs);
        Logger.processInputs("Climber", climberInputs);

        if (climberControlLoopConfigurator.hasChanged()) {
            climberIO.configureControlLoop(climberControlLoopConfigurator.getConfig());
        }

        handleStateTransitions();
        handleCurrentState();
    }

    private void handleStateTransitions() {
        switch (desiredState) {
            case STOPPED:
                currentState = ClimberCurrentState.STOPPED;
                break;
            case HOME:
                currentState = isAtPosition(config.climberHomePositionRotations)
                    ? ClimberCurrentState.HOME
                    : ClimberCurrentState.TRANSITIONING_TO_HOME;
                break;
            case READY:
                currentState = isAtPosition(config.climberReadyPositionRotations)
                    ? ClimberCurrentState.READY
                    : ClimberCurrentState.TRANSITIONING_TO_READY;
                break;
            case LIFTED:
                currentState = isAtPosition(config.climberLiftedPositionRotations)
                    ? ClimberCurrentState.LIFTED
                    : ClimberCurrentState.TRANSITIONING_TO_LIFTED;
                break;
        }
    }

    private void handleCurrentState() {
        switch (currentState) {
            case STOPPED:
                handleStoppedState();
                break;
            case HOME:
            case TRANSITIONING_TO_HOME:
                handleHomeState();
                break;
            case READY:
            case TRANSITIONING_TO_READY:
                handleReadyState();
                break;
            case LIFTED:
            case TRANSITIONING_TO_LIFTED:
                handleLiftedState();
                break;
        }
    }

    private void handleStoppedState() {
        climberIO.setVoltage(0);
    }

    private void handleHomeState() {
        setClimberPosition(config.climberHomePositionRotations);
    }

    private void handleReadyState() {
        setClimberPosition(config.climberReadyPositionRotations);
    }

    private void handleLiftedState() {
        setClimberPosition(config.climberLiftedPositionRotations);
    }

    private void setClimberPosition(double positionRotations) {
        targetPositionRotations = positionRotations;
        Logger.recordOutput("Climber/positionSetpointRotations", positionRotations);
        climberIO.setPosition(positionRotations);
    }

    private boolean isAtPosition(double positionRotations) {
        return Math.abs(climberInputs.positionRotations - positionRotations) < config.climberPositionToleranceRotations;
    }

    @AutoLogOutput(key = "Climber/currentState")
    public ClimberCurrentState getCurrentState() {
        return currentState;
    }

    @AutoLogOutput(key = "Climber/desiredState")
    public ClimberDesiredState getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(ClimberDesiredState desiredState) {
        this.desiredState = desiredState;
    }

    public double getClimberPositionRotations() {
        return climberInputs.positionRotations;
    }

    @AutoLogOutput(key = "Climber/isAtSetpoint")
    public boolean isAtSetpoint() {
        return isAtPosition(targetPositionRotations);
    }
}
