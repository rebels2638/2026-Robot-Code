package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.configs.KickerConfig;
import frc.robot.lib.util.ConfigLoader;
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
        DISABLED,
        HOME,
        FEEDING,
        KICKING
    }

    public enum KickerDesiredState {
        DISABLED,
        HOME,
        FEEDING,
        KICKING
    }

    // FSM State Variables
    private KickerCurrentState currentState = KickerCurrentState.DISABLED;
    private KickerDesiredState desiredState = KickerDesiredState.DISABLED;

    private final KickerIO kickerIO;
    private final KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();

    private final KickerConfig config;

    private final DashboardMotorControlLoopConfigurator kickerControlLoopConfigurator;

    private double kickerSetpointRPS = 0.0;

    private Kicker() {
        config = ConfigLoader.load("kicker", KickerConfig.class);
        kickerIO = RobotBase.isSimulation() ? new KickerIOSim(config) : new KickerIOTalonFX(config);

        kickerControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Kicker/kickerControlLoop",
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.kickerKP,
                config.kickerKI,
                config.kickerKD,
                config.kickerKS,
                config.kickerKV,
                config.kickerKA
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
            case DISABLED:
                currentState = KickerCurrentState.DISABLED;
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
            case DISABLED:
                handleDISABLEDState();
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

    private void handleDISABLEDState() {
        setKickerVelocity(0);
    }

    private void handleHomeState() {
        setKickerVelocity(0);
    }

    private void handleFeedingState() {
        setKickerVelocity(config.feedingVelocityRPS);
    }

    private void handleKickingState() {
        setKickerVelocity(config.kickingVelocityRPS);
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
        return Math.abs(kickerInputs.velocityRotationsPerSec - kickerSetpointRPS) < config.kickerVelocityToleranceRPS;
    }

    /**
     * Returns true if the kicker is ready for kicking (feeding state and at setpoint).
     */
    public boolean isReadyForKicking() {
        return currentState == KickerCurrentState.FEEDING && isKickerAtSetpoint();
    }
}
