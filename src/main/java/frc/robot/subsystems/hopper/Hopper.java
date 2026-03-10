package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.HopperConfig;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Hopper extends SubsystemBase {
    private static Hopper instance = null;

    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }

    public enum HopperCurrentState {
        DISABLED,
        HOME,
        FEEDING,
        REVERSE
    }

    public enum HopperDesiredState {
        DISABLED,
        HOME,
        FEEDING,
        REVERSE
    }

    private HopperCurrentState currentState = HopperCurrentState.DISABLED;
    private HopperDesiredState desiredState = HopperDesiredState.DISABLED;

    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

    private final HopperConfig config;

    private final DashboardMotorControlLoopConfigurator hopperControlLoopConfigurator;

    private double hopperSetpointRPS = 0.0;

    private Hopper() {
        config = ConfigLoader.load("hopper", HopperConfig.class);
        hopperIO = RobotBase.isSimulation() ? new HopperIOSim(config) : new HopperIOTalonFX(config);

        hopperControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Hopper/hopperControlLoop",
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.hopperKP,
                config.hopperKI,
                config.hopperKD,
                config.hopperKS,
                config.hopperKV,
                config.hopperKA
            )
        );
    }

    @Override
    public void periodic() {
        hopperIO.updateInputs(hopperInputs);
        Logger.processInputs("Hopper", hopperInputs);

        if (hopperControlLoopConfigurator.hasChanged()) {
            hopperIO.configureControlLoop(hopperControlLoopConfigurator.getConfig());
        }

        handleStateTransitions();
        handleCurrentState();
    }

    private void handleStateTransitions() {
        switch (desiredState) {
            case DISABLED:
                currentState = HopperCurrentState.DISABLED;
                break;
            case HOME:
                currentState = HopperCurrentState.HOME;
                break;
            case FEEDING:
                currentState = HopperCurrentState.FEEDING;
                break;
            case REVERSE:
                currentState = HopperCurrentState.REVERSE;
                break;
        }
    }

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
            case REVERSE:
                handleReverseState();
                break;
        }
    }

    private void handleDISABLEDState() {
        setHopperVelocity(0);
    }

    private void handleHomeState() {
        setHopperVelocity(0);
    }

    private void handleFeedingState() {
        setHopperVelocity(config.feedingVelocityRPS);
    }

    private void handleReverseState() {
        setHopperVelocity(config.reverseVelocityRPS);
    }

    private void setHopperVelocity(double velocityRotationsPerSec) {
        hopperSetpointRPS = velocityRotationsPerSec;
        Logger.recordOutput("Hopper/velocitySetpointRPS", velocityRotationsPerSec);
        hopperIO.setVelocity(velocityRotationsPerSec);
    }

    @AutoLogOutput(key = "Hopper/currentState")
    public HopperCurrentState getCurrentState() {
        return currentState;
    }

    @AutoLogOutput(key = "Hopper/desiredState")
    public HopperDesiredState getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(HopperDesiredState desiredState) {
        this.desiredState = desiredState;
    }

    public double getHopperVelocityRotationsPerSec() {
        return hopperInputs.velocityRotationsPerSec;
    }

    @AutoLogOutput(key = "Hopper/isHopperAtSetpoint")
    public boolean isHopperAtSetpoint() {
        return Math.abs(hopperInputs.velocityRotationsPerSec - hopperSetpointRPS) < config.hopperVelocityToleranceRPS;
    }
}
