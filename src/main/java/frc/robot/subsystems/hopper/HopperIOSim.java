package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.configs.HopperConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public class HopperIOSim implements HopperIO {
    private final DCMotor hopperMotorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim hopperSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(hopperMotorModel, 0.00015, 1.53),
            hopperMotorModel
        );

    private PIDController hopperFeedback;
    private SimpleMotorFeedforward hopperFeedforward;

    private boolean isHopperClosedLoop = true;
    private double desiredHopperVelocityRotationsPerSec = 0;

    private double lastTimeInputs = Timer.getTimestamp();

    public HopperIOSim(HopperConfig config) {
        hopperFeedback = new PIDController(config.hopperKP, config.hopperKI, config.hopperKD);
        hopperFeedforward = new SimpleMotorFeedforward(config.hopperKS, config.hopperKV, config.hopperKA);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        double dt = Timer.getTimestamp() - lastTimeInputs;
        lastTimeInputs = Timer.getTimestamp();

        if (isHopperClosedLoop) {
            hopperSim.setInputVoltage(
                MathUtil.clamp(
                    hopperFeedforward.calculate(desiredHopperVelocityRotationsPerSec) +
                    hopperFeedback.calculate(hopperSim.getAngularVelocityRadPerSec()),
                    -12,
                    12
                )
            );
        }

        hopperSim.update(dt);

        inputs.velocityRotationsPerSec = hopperSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = hopperSim.getInputVoltage();
        inputs.torqueCurrent = hopperSim.getCurrentDrawAmps();
        inputs.temperatureFahrenheit = 70.0;
    }

    @Override
    public void setVelocity(double velocityRotationsPerSec) {
        hopperFeedback.setSetpoint(velocityRotationsPerSec);
        desiredHopperVelocityRotationsPerSec = velocityRotationsPerSec;
        isHopperClosedLoop = true;
    }

    @Override
    public void setVoltage(double voltage) {
        hopperSim.setInputVoltage(voltage);
        isHopperClosedLoop = false;
    }

    @Override
    public void configureControlLoop(MotorControlLoopConfig config) {
        hopperFeedback.setP(config.kP());
        hopperFeedback.setI(config.kI());
        hopperFeedback.setD(config.kD());

        hopperFeedforward.setKs(config.kS());
        hopperFeedforward.setKv(config.kV());
        hopperFeedforward.setKa(config.kA());
    }
}
