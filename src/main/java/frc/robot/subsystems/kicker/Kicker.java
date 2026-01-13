package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.kicker.KickerConfigBase;
import frc.robot.constants.kicker.comp.KickerConfigComp;
import frc.robot.constants.kicker.proto.KickerConfigProto;
import frc.robot.constants.kicker.sim.KickerConfigSim;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Kicker extends SubsystemBase {
    private static Kicker instance = null;

    public static Kicker getInstance() {
        if (instance == null) {
            instance = new Kicker();
        }
        return instance;
    }

    // FSM State Enums
    public enum KickerCurrentState {
        STOPPED,
        HOME,
        FEEDING,
        KICKING
    }

    public enum KickerDesiredState {
        STOPPED,
        HOME,
        FEEDING,
        KICKING
    }

    // FSM State Variables
    private KickerCurrentState currentState = KickerCurrentState.STOPPED;
    private KickerDesiredState desiredState = KickerDesiredState.STOPPED;

    private final KickerIO kickerIO;
    private final KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();

    private final KickerConfigBase config;

    private final DashboardMotorControlLoopConfigurator kickerControlLoopConfigurator;

    private double kickerSetpointRPS = 0.0;

    private Kicker() {
        switch (Constants.currentMode) {
            case COMP:
                config = KickerConfigComp.getInstance();
                kickerIO = new KickerIOTalonFX(config);
                break;

            case PROTO:
                config = KickerConfigProto.getInstance();
                kickerIO = new KickerIOTalonFX(config);
                break;

            case SIM:
                config = KickerConfigSim.getInstance();
                kickerIO = new KickerIOSim(config);
                break;

            case REPLAY:
                config = KickerConfigComp.getInstance();
                kickerIO = new KickerIOTalonFX(config);
                break;

            default:
                config = KickerConfigComp.getInstance();
                kickerIO = new KickerIOTalonFX(config);
                break;
        }

        kickerControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Kicker/kickerControlLoop",
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.getKickerKP(),
                config.getKickerKI(),
                config.getKickerKD(),
                config.getKickerKS(),
                config.getKickerKV(),
                config.getKickerKA()
            )
        );
    }

    @Override
    public void periodic() {
        kickerIO.updateInputs(kickerInputs);
        Logger.processInputs("Kicker", kickerInputs);

        // Handle control loop configuration updates
        if (kickerControlLoopConfigurator.hasChanged()) {
            kickerIO.configureControlLoop(kickerControlLoopConfigurator.getConfig());
        }

        // FSM processing
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
                currentState = KickerCurrentState.STOPPED;
                break;

            case HOME:
                currentState = KickerCurrentState.HOME;
                break;

            case FEEDING:
                currentState = KickerCurrentState.FEEDING;
                break;

            case KICKING:
                currentState = KickerCurrentState.KICKING;
                break;
        }
    }

    /**
     * Executes behavior for the current state.
     */
    private void handleCurrentState() {
        switch (currentState) {
            case STOPPED:
                handleStoppedState();
                break;
            case HOME:
                handleHomeState();
                break;
            case FEEDING:
                handleFeedingState();
                break;
            case KICKING:
                handleKickingState();
                break;
        }
    }

    private void handleStoppedState() {
        setKickerVelocity(0);
    }

    private void handleHomeState() {
        setKickerVelocity(0);
    }

    private void handleFeedingState() {
        setKickerVelocity(config.getFeedingVelocityRPS());
    }

    private void handleKickingState() {
        setKickerVelocity(config.getKickingVelocityRPS());
    }

    private void setKickerVelocity(double velocityRotationsPerSec) {
        kickerSetpointRPS = velocityRotationsPerSec;
        Logger.recordOutput("Kicker/velocitySetpointRPS", velocityRotationsPerSec);
        kickerIO.setVelocity(velocityRotationsPerSec);
    }

    // State getters/setters
    @AutoLogOutput(key = "Kicker/currentState")
    public KickerCurrentState getCurrentState() {
        return currentState;
    }

    @AutoLogOutput(key = "Kicker/desiredState")
    public KickerDesiredState getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(KickerDesiredState desiredState) {
        this.desiredState = desiredState;
    }

    // Mechanism getters
    public double getKickerVelocityRotationsPerSec() {
        return kickerInputs.velocityRotationsPerSec;
    }

    // Setpoint check methods
    @AutoLogOutput(key = "Kicker/isKickerAtSetpoint")
    public boolean isKickerAtSetpoint() {
        return Math.abs(kickerInputs.velocityRotationsPerSec - kickerSetpointRPS) < config.getKickerVelocityToleranceRPS();
    }

    /**
     * Returns true if the kicker is ready for kicking (feeding state and at setpoint).
     */
    public boolean isReadyForKicking() {
        return currentState == KickerCurrentState.FEEDING && isKickerAtSetpoint();
    }
}
