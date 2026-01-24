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

    public enum IntakeSetpoint {
        OFF(0.0),
        INTAKING(Double.NaN),
        OUTTAKING(Double.NaN);

        private final double rps;
        IntakeSetpoint(double rps) {
            this.rps = rps;
        }

        public double getRps() {
            return rps;
        }
    }

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

    }

    private void setIntakeVelocity(double velocityRotationsPerSec) {
        intakeSetpointRPS = velocityRotationsPerSec;
        Logger.recordOutput("Intake/velocitySetpointRPS", velocityRotationsPerSec);
        intakeIO.setVelocity(velocityRotationsPerSec);
    }

    public void setSetpoint(IntakeSetpoint setpoint) {
        double targetRps = setpoint == IntakeSetpoint.INTAKING
            ? config.intakingVelocityRPS
            : setpoint == IntakeSetpoint.OUTTAKING ? config.outtakingVelocityRPS : setpoint.getRps();
        setIntakeVelocity(targetRps);
    }

    public double getIntakeVelocityRotationsPerSec() {
        return intakeInputs.velocityRotationsPerSec;
    }

    @AutoLogOutput(key = "Intake/isIntakeAtSetpoint")
    public boolean isIntakeAtSetpoint() {
        return Math.abs(intakeInputs.velocityRotationsPerSec - intakeSetpointRPS) < config.intakeVelocityToleranceRPS;
    }
}
