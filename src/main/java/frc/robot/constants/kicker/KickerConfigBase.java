package frc.robot.constants.kicker;

public abstract class KickerConfigBase {
    public abstract String getCanBusName();

    // Kicker motor config (velocity control)
    public abstract int getKickerCanId();
    public abstract boolean getIsKickerInverted();

    public abstract double getKickerSupplyCurrentLimit();
    public abstract double getKickerSupplyCurrentLimitLowerTime();
    public abstract double getKickerSupplyCurrentLimitLowerLimit();
    public abstract double getKickerStatorCurrentLimit();
    public abstract double getKickerPeakForwardTorqueCurrent();
    public abstract double getKickerPeakReverseTorqueCurrent();

    public abstract double getKickerKS();
    public abstract double getKickerKV();
    public abstract double getKickerKA();
    public abstract double getKickerKP();
    public abstract double getKickerKI();
    public abstract double getKickerKD();

    public abstract double getKickerMotorToOutputShaftRatio();

    // Velocities for different states
    public abstract double getFeedingVelocityRPS();
    public abstract double getKickingVelocityRPS();

    // Tolerance
    public abstract double getKickerVelocityToleranceRPS();
}
