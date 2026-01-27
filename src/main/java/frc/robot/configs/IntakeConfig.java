package frc.robot.configs;

public class IntakeConfig {
    public String canBusName;

    public int intakeCanId;
    public boolean isIntakeInverted;

    public double intakeStatorCurrentLimit;
    public double intakePeakForwardTorqueCurrent;
    public double intakePeakReverseTorqueCurrent;

    public double intakeKS;
    public double intakeKV;
    public double intakeKA;
    public double intakeKP;
    public double intakeKI;
    public double intakeKD;

    public double intakeMotorToOutputShaftRatio;

    public double intakingVelocityRPS;
    public double outtakingVelocityRPS;

    public double intakeVelocityToleranceRPS;
}
