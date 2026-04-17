package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.HopperConfig;
import frc.robot.constants.Constants;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Hopper extends SubsystemBase {
    private static final double HOPPER_VOLTAGE_SETPOINT_TOLERANCE_VOLTS = 0.25;

    private enum HopperControlMode {
        VELOCITY,
        VOLTAGE
    }

    private static Hopper instance = null;

    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }

    public enum HopperSetpoint {
        OFF,
        FEEDING,
        FEEDING_IDLE,
        REVERSE
    }

    public enum UnjamState {
        IDLE,
        UNJAMMING,
        COOLDOWN
    }

    static final class UnjamTransition {
        final UnjamState nextState;
        final int nextSegmentIndex;
        final boolean resetSegmentTimer;
        final boolean resetCooldownTimer;

        UnjamTransition(
            UnjamState nextState,
            int nextSegmentIndex,
            boolean resetSegmentTimer,
            boolean resetCooldownTimer
        ) {
            this.nextState = nextState;
            this.nextSegmentIndex = nextSegmentIndex;
            this.resetSegmentTimer = resetSegmentTimer;
            this.resetCooldownTimer = resetCooldownTimer;
        }
    }

    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

    private final HopperConfig config;

    private final DashboardMotorControlLoopConfigurator hopperControlLoopConfigurator;
    private boolean pendingHopperControlLoopConfigApply = false;
    private final boolean enableConnectionAlerts;
    private final Alert hopperDisconnectedAlert;

    private HopperControlMode hopperControlMode = HopperControlMode.VELOCITY;
    private double hopperSetpointRPS = 0.0;
    private double hopperSetpointVolts = 0.0;

    private HopperSetpoint commandedSetpoint = HopperSetpoint.OFF;
    private UnjamState unjamState = UnjamState.IDLE;
    private int unjamSegmentIndex = 0;
    private double unjamSegmentStartTimestamp = 0.0;
    private double cooldownStartTimestamp = 0.0;
    private final Debouncer stallDebouncer;

    private final DoublePublisher hopperTorqueCurrentPublisher =
        NetworkTableInstance.getDefault().getDoubleTopic("Hopper/torqueCurrent").publish();
    private final DoublePublisher hopperMeasuredVelocityPublisher =
        NetworkTableInstance.getDefault().getDoubleTopic("Hopper/measuredVelocityRPS").publish();

    private Hopper() {
        boolean useSimulation = Constants.shouldUseSimulation(Constants.SimOnlySubsystems.HOPPER);
        config = ConfigLoader.load(
            "hopper",
            ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.HOPPER),
            HopperConfig.class
        );
        hopperIO = useSimulation ? new HopperIOSim(config) : new HopperIOTalonFX(config);
        enableConnectionAlerts = !useSimulation && Constants.currentMode != Constants.Mode.REPLAY;
        hopperDisconnectedAlert = new Alert("Hopper motor is disconnected.", AlertType.kWarning);

        stallDebouncer = new Debouncer(config.jamDetectionDebounceSeconds, Debouncer.DebounceType.kRising);

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

        hopperTorqueCurrentPublisher.set(hopperInputs.torqueCurrent);
        hopperMeasuredVelocityPublisher.set(hopperInputs.velocityRotationsPerSec);

        hopperDisconnectedAlert.set(enableConnectionAlerts && !hopperInputs.hopperMotorConnected);

        updateUnjamStateMachine();
        applyEffectiveSetpoint();

        pendingHopperControlLoopConfigApply |= hopperControlLoopConfigurator.hasChanged();
        if (DriverStation.isDisabled() && pendingHopperControlLoopConfigApply) {
            hopperIO.configureControlLoop(hopperControlLoopConfigurator.getConfig());
            pendingHopperControlLoopConfigApply = false;
        }
    }

    private void updateUnjamStateMachine() {
        double now = Timer.getTimestamp();
        boolean gateActive = commandedSetpoint == HopperSetpoint.FEEDING
            && config.jamDetectionEnabled
            && hopperInputs.hopperMotorConnected;

        boolean rawStall = isStallCandidate(
            hopperInputs.torqueCurrent,
            hopperInputs.velocityRotationsPerSec,
            config.jamDetectionCurrentThresholdAmps,
            config.jamDetectionMaxVelocityRPS
        );
        // Debouncer only sees the real signal while gate is active AND we're in IDLE;
        // otherwise feed false so it clears and re-arms cleanly when we return to IDLE.
        boolean debouncerInput = gateActive && unjamState == UnjamState.IDLE && rawStall;
        boolean debouncedStall = stallDebouncer.calculate(debouncerInput);

        double segmentElapsed = now - unjamSegmentStartTimestamp;
        double cooldownElapsed = now - cooldownStartTimestamp;

        UnjamTransition transition = resolveNextUnjamState(
            unjamState,
            unjamSegmentIndex,
            debouncedStall,
            segmentElapsed,
            cooldownElapsed,
            config.unjamCycleCount,
            config.unjamSegmentDurationSeconds,
            config.unjamCooldownSeconds,
            gateActive
        );

        unjamState = transition.nextState;
        unjamSegmentIndex = transition.nextSegmentIndex;
        if (transition.resetSegmentTimer) {
            unjamSegmentStartTimestamp = now;
        }
        if (transition.resetCooldownTimer) {
            cooldownStartTimestamp = now;
        }

        Logger.recordOutput("Hopper/unjamState", unjamState.name());
        Logger.recordOutput("Hopper/unjamSegmentIndex", unjamSegmentIndex);
        Logger.recordOutput("Hopper/stallDetectedRaw", rawStall);
        Logger.recordOutput("Hopper/stallDetectedDebounced", debouncedStall);
    }

    static boolean isStallCandidate(
        double torqueCurrentAmps,
        double velocityRPS,
        double currentThresholdAmps,
        double maxVelocityRPS
    ) {
        return Math.abs(torqueCurrentAmps) > currentThresholdAmps
            && Math.abs(velocityRPS) < maxVelocityRPS;
    }

    static UnjamTransition resolveNextUnjamState(
        UnjamState currentState,
        int currentSegmentIndex,
        boolean debouncedStall,
        double segmentElapsedSeconds,
        double cooldownElapsedSeconds,
        int unjamCycleCount,
        double unjamSegmentDurationSeconds,
        double unjamCooldownSeconds,
        boolean gateActive
    ) {
        if (!gateActive) {
            return new UnjamTransition(UnjamState.IDLE, 0, false, false);
        }
        switch (currentState) {
            case IDLE:
                if (debouncedStall) {
                    return new UnjamTransition(UnjamState.UNJAMMING, 0, true, false);
                }
                return new UnjamTransition(UnjamState.IDLE, 0, false, false);
            case UNJAMMING:
                if (segmentElapsedSeconds >= unjamSegmentDurationSeconds) {
                    int nextIndex = currentSegmentIndex + 1;
                    if (nextIndex >= 2 * unjamCycleCount) {
                        return new UnjamTransition(UnjamState.COOLDOWN, 0, false, true);
                    }
                    return new UnjamTransition(UnjamState.UNJAMMING, nextIndex, true, false);
                }
                return new UnjamTransition(UnjamState.UNJAMMING, currentSegmentIndex, false, false);
            case COOLDOWN:
                if (cooldownElapsedSeconds >= unjamCooldownSeconds) {
                    return new UnjamTransition(UnjamState.IDLE, 0, false, false);
                }
                return new UnjamTransition(UnjamState.COOLDOWN, currentSegmentIndex, false, false);
            default:
                return new UnjamTransition(UnjamState.IDLE, 0, false, false);
        }
    }

    private void applyEffectiveSetpoint() {
        HopperSetpoint effective = commandedSetpoint;
        if (unjamState == UnjamState.UNJAMMING) {
            effective = (unjamSegmentIndex % 2 == 0) ? HopperSetpoint.REVERSE : HopperSetpoint.FEEDING;
        }
        applySetpointToMotor(effective);
    }

    public void setHopperVelocity(double velocityRotationsPerSec) {
        hopperControlMode = HopperControlMode.VELOCITY;
        hopperSetpointRPS = velocityRotationsPerSec;
        hopperSetpointVolts = Double.NaN;
        Logger.recordOutput("Hopper/controlMode", hopperControlMode.name());
        Logger.recordOutput("Hopper/velocitySetpointRPS", velocityRotationsPerSec);
        Logger.recordOutput("Hopper/voltageSetpointVolts", Double.NaN);
        hopperIO.setVelocity(velocityRotationsPerSec);
    }

    public void setHopperVoltage(double voltage) {
        hopperControlMode = HopperControlMode.VOLTAGE;
        hopperSetpointRPS = Double.NaN;
        hopperSetpointVolts = voltage;
        Logger.recordOutput("Hopper/controlMode", hopperControlMode.name());
        Logger.recordOutput("Hopper/velocitySetpointRPS", Double.NaN);
        Logger.recordOutput("Hopper/voltageSetpointVolts", voltage);
        hopperIO.setVoltage(voltage);
    }

    public void setSetpoint(HopperSetpoint setpoint) {
        if (setpoint != commandedSetpoint) {
            unjamState = UnjamState.IDLE;
            unjamSegmentIndex = 0;
            stallDebouncer.calculate(false);
        }
        commandedSetpoint = setpoint;
    }

    private void applySetpointToMotor(HopperSetpoint setpoint) {
        switch (setpoint) {
            case OFF:
                setHopperVelocity(0.0);
                break;
            case FEEDING:
                setHopperVelocity(config.feedingVelocityRPS);
                break;
            case FEEDING_IDLE:
                setHopperVoltage(config.feedingIdleVoltage);
                break;
            case REVERSE:
                setHopperVelocity(config.reverseVelocityRPS);
                break;
        }
    }

    public double getHopperVelocityRotationsPerSec() {
        return hopperInputs.velocityRotationsPerSec;
    }

    public void enableEStop() {
        hopperIO.enableEStop();
    }

    public void disableEStop() {
        hopperIO.disableEStop();
    }

    @AutoLogOutput(key = "Hopper/isHopperAtSetpoint")
    public boolean isHopperAtSetpoint() {
        if (!hopperInputs.hopperMotorConnected) {
            return false;
        }

        return hopperControlMode == HopperControlMode.VELOCITY
            ? Math.abs(hopperInputs.velocityRotationsPerSec - hopperSetpointRPS) < config.hopperVelocityToleranceRPS
            : Math.abs(hopperInputs.appliedVolts - hopperSetpointVolts) < HOPPER_VOLTAGE_SETPOINT_TOLERANCE_VOLTS;
    }

    @AutoLogOutput(key = "Hopper/isHopperMotorConnected")
    public boolean isHopperMotorConnected() {
        return hopperInputs.hopperMotorConnected;
    }
}
