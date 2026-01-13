package frc.robot.constants.kicker.comp;

import frc.robot.constants.kicker.KickerConfigBase;

public class KickerConfigComp extends KickerConfigBase {

    public static KickerConfigComp instance = null;
    public static KickerConfigComp getInstance() {
        if (instance == null) {
            instance = new KickerConfigComp();
        }
        return instance;
    }

    private KickerConfigComp() {}

    @Override
    public String getCanBusName() {
        return "drivetrain";
    }

    // Kicker motor config (reusing indexer CAN ID 20)
    @Override
    public int getKickerCanId() {
        return 20;
    }

    @Override
    public boolean getIsKickerInverted() {
        return true;
    }

    @Override
    public double getKickerSupplyCurrentLimit() {
        return 90.0;
    }

    @Override
    public double getKickerSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getKickerSupplyCurrentLimitLowerLimit() {
        return 90.0;
    }

    @Override
    public double getKickerStatorCurrentLimit() {
        return 90.0;
    }

    @Override
    public double getKickerPeakForwardTorqueCurrent() {
        return 90.0;
    }

    @Override
    public double getKickerPeakReverseTorqueCurrent() {
        return -90.0;
    }

    @Override
    public double getKickerKS() {
        return 0.0;
    }

    @Override
    public double getKickerKV() {
        return 0.3;
    }

    @Override
    public double getKickerKA() {
        return 0.0;
    }

    @Override
    public double getKickerKP() {
        return 0.1;
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
        return 3.0;
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
