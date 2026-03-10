package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.configs.IntakeConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public class IntakeIOSim implements IntakeIO {
    private final DCMotor intakeMotorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim intakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(intakeMotorModel, 0.00015, 1.53),
            intakeMotorModel
        );

    private PIDController intakeFeedback;
    private SimpleMotorFeedforward intakeFeedforward;

    private boolean isIntakeClosedLoop = true;
    private double desiredIntakeVelocityRotationsPerSec = 0;

    private double lastTimeInputs = Timer.getTimestamp();

    public IntakeIOSim(IntakeConfig config) {
        intakeFeedback = new PIDController(config.intakeKP, config.intakeKI, config.intakeKD);
        intakeFeedforward = new SimpleMotorFeedforward(config.intakeKS, config.intakeKV, config.intakeKA);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double dt = Timer.getTimestamp() - lastTimeInputs;
        lastTimeInputs = Timer.getTimestamp();

        if (isIntakeClosedLoop) {
            intakeSim.setInputVoltage(
                MathUtil.clamp(
                    intakeFeedforward.calculate(desiredIntakeVelocityRotationsPerSec) +
                    intakeFeedback.calculate(intakeSim.getAngularVelocityRadPerSec()),
                    -12,
                    12
                )
            );
        }

        intakeSim.update(dt);

        inputs.velocityRotationsPerSec = intakeSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = intakeSim.getInputVoltage();
        inputs.torqueCurrent = intakeSim.getCurrentDrawAmps();
        inputs.temperatureFahrenheit = 70.0;
    }

    @Override
    public void setVelocity(double velocityRotationsPerSec) {
        intakeFeedback.setSetpoint(velocityRotationsPerSec);
        desiredIntakeVelocityRotationsPerSec = velocityRotationsPerSec;
        isIntakeClosedLoop = true;
    }

    @Override
    public void setVoltage(double voltage) {
        intakeSim.setInputVoltage(voltage);
        isIntakeClosedLoop = false;
    }

    @Override
    public void configureControlLoop(MotorControlLoopConfig config) {
        intakeFeedback.setP(config.kP());
        intakeFeedback.setI(config.kI());
        intakeFeedback.setD(config.kD());

        intakeFeedforward.setKs(config.kS());
        intakeFeedforward.setKv(config.kV());
        intakeFeedforward.setKa(config.kA());
    }
}
