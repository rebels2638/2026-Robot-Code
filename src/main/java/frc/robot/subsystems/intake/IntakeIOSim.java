package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.configs.IntakeConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;
import frc.robot.sim.MapleSimManager;

public class IntakeIOSim implements IntakeIO {
    private static final String MAPLE_GAME_PIECE_TYPE = "Fuel";
    private static final double MAPLE_INTAKE_WIDTH_METERS = 0.70;
    private static final double MAPLE_INTAKE_EXTENSION_METERS = 0.20;
    private static final int MAPLE_INTAKE_CAPACITY = 1;
    private static final double MIN_INTAKE_SPEED_RPS = 1.0;
    private static final double MIN_INTAKE_VOLTAGE = 0.1;
    private static final double DEPLOYED_PIVOT_MARGIN_ROTATIONS = 0.01;

    private final DCMotor rollerMotorModel = DCMotor.getKrakenX60Foc(1);
    private final DCMotor pivotMotorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rollerMotorModel, 0.00015, 1.53),
            rollerMotorModel
        );

    private final DCMotorSim pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(pivotMotorModel, 0.00015, 21.428),
            pivotMotorModel
        );

    private PIDController rollerFeedback;
    private SimpleMotorFeedforward rollerFeedforward;
    private ProfiledPIDController pivotFeedback;

    private boolean isRollerClosedLoop = true;
    private boolean isPivotClosedLoop = true;
    private double desiredRollerVelocityRotationsPerSec = 0;
    private boolean isRollerEStopped = false;
    private boolean isPivotEStopped = false;
    private double commandedRollerVoltage = 0.0;
    private boolean mapleIntakeRunning = false;

    private double lastTimeInputs = Timer.getTimestamp();
    private final IntakeConfig config;
    private final IntakeSimulation intakeSimulation;

    public IntakeIOSim(IntakeConfig config) {
        this.config = config;
        rollerFeedback = new PIDController(config.rollerKP, config.rollerKI, config.rollerKD);
        rollerFeedforward = new SimpleMotorFeedforward(config.rollerKS, config.rollerKV, config.rollerKA);
        double pivotMaxVelRadPerSec = config.pivotMaxVelocityRotationsPerSec * 2.0 * Math.PI;
        double pivotMaxAccelRadPerSec2 = config.pivotMaxAccelerationRotationsPerSec2 * 2.0 * Math.PI;
        pivotFeedback =
            new ProfiledPIDController(
                config.pivotKP,
                config.pivotKI,
                config.pivotKD,
                new TrapezoidProfile.Constraints(pivotMaxVelRadPerSec, pivotMaxAccelRadPerSec2)
            );

        pivotSim.setState(config.pivotStartingAngleRotations * 2 * Math.PI, 0);
        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            MAPLE_GAME_PIECE_TYPE,
            MapleSimManager.getInstance().getDriveSimulation(),
            Meters.of(MAPLE_INTAKE_WIDTH_METERS),
            Meters.of(MAPLE_INTAKE_EXTENSION_METERS),
            IntakeSimulation.IntakeSide.FRONT,
            MAPLE_INTAKE_CAPACITY
        );
        intakeSimulation.stopIntake();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double dt = Timer.getTimestamp() - lastTimeInputs;
        lastTimeInputs = Timer.getTimestamp();

        if (isRollerEStopped) {
            rollerSim.setInputVoltage(0);
        } else if (isRollerClosedLoop) {
            rollerSim.setInputVoltage(
                MathUtil.clamp(
                    rollerFeedforward.calculate(desiredRollerVelocityRotationsPerSec) +
                    rollerFeedback.calculate(rollerSim.getAngularVelocityRadPerSec() / (2 * Math.PI)),
                    -12,
                    12
                )
            );
        }

        if (isPivotEStopped) {
            pivotSim.setInputVoltage(0);
        } else if (isPivotClosedLoop) {
            pivotSim.setInputVoltage(
                MathUtil.clamp(
                    pivotFeedback.calculate(pivotSim.getAngularPositionRad()),
                    -12,
                    12
                )
            );
        }

        rollerSim.update(dt);
        pivotSim.update(dt);
        updateMapleIntakeSimulation();

        inputs.rollerMotorConnected = true;
        inputs.rollerVelocityRotationsPerSec = rollerSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.rollerAppliedVolts = rollerSim.getInputVoltage();
        inputs.rollerTorqueCurrent = rollerSim.getCurrentDrawAmps();
        inputs.rollerTemperatureFahrenheit = 70.0;

        inputs.pivotMotorConnected = true;
        inputs.pivotAngleRotations = pivotSim.getAngularPositionRotations();
        inputs.pivotVelocityRotationsPerSec = pivotSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        inputs.pivotAppliedVolts = pivotSim.getInputVoltage();
        inputs.pivotTorqueCurrent = pivotSim.getCurrentDrawAmps();
        inputs.pivotTemperatureFahrenheit = 70.0;

        Logger.recordOutput("Intake/Sim/MapleIntakeRunning", mapleIntakeRunning);
        Logger.recordOutput("Intake/Sim/MapleGamePiecesInIntake", intakeSimulation.getGamePiecesAmount());
    }

    @Override
    public void setVelocity(double velocityRotationsPerSec) {
        if (isRollerEStopped) {
            rollerSim.setInputVoltage(0);
            isRollerClosedLoop = false;
            return;
        }
        rollerFeedback.setSetpoint(velocityRotationsPerSec);
        desiredRollerVelocityRotationsPerSec = velocityRotationsPerSec;
        isRollerClosedLoop = true;
    }

    @Override
    public void setVoltage(double voltage) {
        commandedRollerVoltage = voltage;
        rollerSim.setInputVoltage(isRollerEStopped ? 0 : voltage);
        isRollerClosedLoop = false;
    }

    @Override
    public void setPivotAngle(double angleRotations) {
        if (isPivotEStopped) {
            pivotSim.setInputVoltage(0);
            isPivotClosedLoop = false;
            return;
        }
        double clampedAngle = MathUtil.clamp(angleRotations,
            config.pivotMinAngleRotations,
            config.pivotMaxAngleRotations);
        pivotFeedback.setGoal(clampedAngle * (2 * Math.PI));
        isPivotClosedLoop = true;
    }

    @Override
    public void configureControlLoop(MotorControlLoopConfig config) {
        rollerFeedback.setP(config.kP());
        rollerFeedback.setI(config.kI());
        rollerFeedback.setD(config.kD());

        rollerFeedforward.setKs(config.kS());
        rollerFeedforward.setKv(config.kV());
        rollerFeedforward.setKa(config.kA());
    }

    @Override
    public void configurePivotControlLoop(MotorControlLoopConfig config) {
        pivotFeedback.setP(config.kP());
        pivotFeedback.setI(config.kI());
        pivotFeedback.setD(config.kD());
    }

    @Override
    public void enableRollerEStop() {
        isRollerEStopped = true;
        commandedRollerVoltage = 0;
        rollerSim.setInputVoltage(0);
        isRollerClosedLoop = false;
    }

    @Override
    public void disableRollerEStop() {
        isRollerEStopped = false;
    }

    @Override
    public void enablePivotEStop() {
        isPivotEStopped = true;
        pivotSim.setInputVoltage(0);
        isPivotClosedLoop = false;
    }

    @Override
    public void disablePivotEStop() {
        isPivotEStopped = false;
    }

    private void updateMapleIntakeSimulation() {
        boolean shouldRun = shouldRunMapleIntake();
        if (shouldRun && !mapleIntakeRunning) {
            intakeSimulation.startIntake();
            mapleIntakeRunning = true;
        } else if (!shouldRun && mapleIntakeRunning) {
            intakeSimulation.stopIntake();
            mapleIntakeRunning = false;
        }

        if (isOuttaking()) {
            intakeSimulation.obtainGamePieceFromIntake();
        }
    }

    private boolean shouldRunMapleIntake() {
        if (isRollerEStopped || isPivotEStopped) {
            return false;
        }

        boolean pivotDeployed =
            pivotSim.getAngularPositionRotations() > config.pivotUpAngleRotations + DEPLOYED_PIVOT_MARGIN_ROTATIONS;
        if (!pivotDeployed) {
            return false;
        }

        return isRollerClosedLoop
            ? desiredRollerVelocityRotationsPerSec > MIN_INTAKE_SPEED_RPS
            : commandedRollerVoltage > MIN_INTAKE_VOLTAGE;
    }

    private boolean isOuttaking() {
        if (isRollerEStopped) {
            return false;
        }

        return isRollerClosedLoop
            ? desiredRollerVelocityRotationsPerSec < -MIN_INTAKE_SPEED_RPS
            : commandedRollerVoltage < -MIN_INTAKE_VOLTAGE;
    }
}
