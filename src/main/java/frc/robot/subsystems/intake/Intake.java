package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.IntakeConfig;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public enum IntakeCurrentState {
        DISABLED,
        HOME,
        INTAKING,
        OUTTAKING
    }

    public enum IntakeDesiredState {
        DISABLED,
        HOME,
        INTAKING,
        OUTTAKING
    }

    private IntakeCurrentState currentState = IntakeCurrentState.DISABLED;
    private IntakeDesiredState desiredState = IntakeDesiredState.DISABLED;

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    private final IntakeConfig config;

    private final DashboardMotorControlLoopConfigurator intakeControlLoopConfigurator;

    private double intakeSetpointRPS = 0.0;

    private Intake() {
        config = ConfigLoader.load("intake", IntakeConfig.class);
        intakeIO = RobotBase.isSimulation() ? new IntakeIOSim(config) : new IntakeIOTalonFX(config);

        intakeControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Intake/intakeControlLoop",
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.intakeKP,
                config.intakeKI,
                config.intakeKD,
                config.intakeKS,
                config.intakeKV,
                config.intakeKA
            )
        );
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Intake", intakeInputs);

        if (intakeControlLoopConfigurator.hasChanged()) {
            intakeIO.configureControlLoop(intakeControlLoopConfigurator.getConfig());
        }

        handleStateTransitions();
        handleCurrentState();
    }

    private void handleStateTransitions() {
        switch (desiredState) {
            case DISABLED:
                currentState = IntakeCurrentState.DISABLED;
                break;
            case HOME:
                currentState = IntakeCurrentState.HOME;
                break;
            case INTAKING:
                currentState = IntakeCurrentState.INTAKING;
                break;
            case OUTTAKING:
                currentState = IntakeCurrentState.OUTTAKING;
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
            case INTAKING:
                handleIntakingState();
                break;
            case OUTTAKING:
                handleOuttakingState();
                break;
        }
    }

    private void handleDISABLEDState() {
        setIntakeVelocity(0);
    }

    private void handleHomeState() {
        setIntakeVelocity(0);
    }

    private void handleIntakingState() {
        setIntakeVelocity(config.intakingVelocityRPS);
    }

    private void handleOuttakingState() {
        setIntakeVelocity(config.outtakingVelocityRPS);
    }

    private void setIntakeVoltage(double voltage) {
        Logger.recordOutput("Intake/voltageSetpoint", voltage);
        intakeIO.setVoltage(voltage);
    }
    
    private void setIntakeVelocity(double velocityRotationsPerSec) {
        intakeSetpointRPS = velocityRotationsPerSec;
        Logger.recordOutput("Intake/velocitySetpointRPS", velocityRotationsPerSec);
        intakeIO.setVelocity(velocityRotationsPerSec);
    }

    @AutoLogOutput(key = "Intake/currentState")
    public IntakeCurrentState getCurrentState() {
        return currentState;
    }

    @AutoLogOutput(key = "Intake/desiredState")
    public IntakeDesiredState getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(IntakeDesiredState desiredState) {
        this.desiredState = desiredState;
    }

    public double getIntakeVelocityRotationsPerSec() {
        return intakeInputs.velocityRotationsPerSec;
    }

    @AutoLogOutput(key = "Intake/isIntakeAtSetpoint")
    public boolean isIntakeAtSetpoint() {
        return Math.abs(intakeInputs.velocityRotationsPerSec - intakeSetpointRPS) < config.intakeVelocityToleranceRPS;
    }
}
