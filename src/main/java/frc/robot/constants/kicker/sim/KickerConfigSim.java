package frc.robot.constants.kicker.sim;

import frc.robot.constants.kicker.KickerConfigBase;

public class KickerConfigSim extends KickerConfigBase {

    public static KickerConfigSim instance = null;
    public static KickerConfigSim getInstance() {
        if (instance == null) {
            instance = new KickerConfigSim();
        }
        return instance;
    }

    private KickerConfigSim() {}

    @Override
    public String getCanBusName() {
        return "rio";
    }

    @Override
    public int getKickerCanId() {
        return 23;
    }

    @Override
    public boolean getIsKickerInverted() {
        return false;
    }

    @Override
    public double getKickerSupplyCurrentLimit() {
        return 30.0;
    }

    @Override
    public double getKickerSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getKickerSupplyCurrentLimitLowerLimit() {
        return 25.0;
    }

    @Override
    public double getKickerStatorCurrentLimit() {
        return 40.0;
    }

    @Override
    public double getKickerPeakForwardTorqueCurrent() {
        return 40.0;
    }

    @Override
    public double getKickerPeakReverseTorqueCurrent() {
        return -40.0;
    }

    @Override
    public double getKickerKS() {
        return 0;
    }

    @Override
    public double getKickerKV() {
        return 0.03;
    }

    @Override
    public double getKickerKA() {
        return 0;
    }

    @Override
    public double getKickerKP() {
        return 0.02;
    }

    @Override
    public double getKickerKI() {
        return 0.0;
    }

    @Override
    public double getKickerKD() {
        return 0.0;
    }

    @Override
    public double getKickerMotorToOutputShaftRatio() {
        return 5.0;
    }

    @Override
    public double getFeedingVelocityRPS() {
        return 35.0;
    }

    @Override
    public double getKickingVelocityRPS() {
        return 35.0;
    }

    @Override
    public double getKickerVelocityToleranceRPS() {
        return 5.0;
    }
}
