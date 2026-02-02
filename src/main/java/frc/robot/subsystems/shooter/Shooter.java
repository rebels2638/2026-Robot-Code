package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ShooterConfig;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Shooter extends SubsystemBase {
    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public enum HoodSetpoint {
        HOME(Rotation2d.fromDegrees(25.0)),
        DYNAMIC(null);

        private final Rotation2d angle;
        HoodSetpoint(Rotation2d angle) {
            this.angle = angle;
        }

        public Rotation2d getAngle() {
            return angle;
        }
    }

    public enum FlywheelSetpoint {
        OFF(0.0),
        DYNAMIC(Double.NaN);

        private final double rps;
        FlywheelSetpoint(double rps) {
            this.rps = rps;
        }

        public double getRps() {
            return rps;
        }
    }

    // Setpoint Suppliers (set by Superstructure)
    private Supplier<Rotation2d> hoodAngleSupplier = () -> Rotation2d.fromDegrees(45.0);
    private Supplier<Double> flywheelRPSSupplier = () -> 0.0;

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final ShooterConfig config;
    
    private final DashboardMotorControlLoopConfigurator hoodControlLoopConfigurator;
    private final DashboardMotorControlLoopConfigurator flywheelControlLoopConfigurator;

    private double hoodSetpointRotations = 0.0;
    private double flywheelSetpointRPS = 0.0;

    private Shooter() {
        boolean useSimulation = Constants.shouldUseSimulation(Constants.SimOnlySubsystems.SHOOTER);
        config = ConfigLoader.load(
            "shooter",
            ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.SHOOTER),
            ShooterConfig.class
        );
        shooterIO = useSimulation ? new ShooterIOSim(config) : new ShooterIOTalonFX(config);

        hoodControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Shooter/hoodControlLoop", 
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.hoodKP,
                config.hoodKI,
                config.hoodKD,
                config.hoodKS,
                config.hoodKV,
                config.hoodKA
            )
        );
        flywheelControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Shooter/flywheelControlLoop", 
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.flywheelKP,
                config.flywheelKI,
                config.flywheelKD,
                config.flywheelKS,
                config.flywheelKV,
                config.flywheelKA
            )
        );
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);

        // Handle control loop configuration updates
        if (hoodControlLoopConfigurator.hasChanged()) {
            shooterIO.configureHoodControlLoop(hoodControlLoopConfigurator.getConfig());
        }
        if (flywheelControlLoopConfigurator.hasChanged()) {
            shooterIO.configureFlywheelControlLoop(flywheelControlLoopConfigurator.getConfig());
        }

    }

    // Supplier setters (called by Superstructure)
    public void setHoodAngleSupplier(Supplier<Rotation2d> supplier) {
        this.hoodAngleSupplier = supplier;
    }

    public void setFlywheelRPSSupplier(Supplier<Double> supplier) {
        this.flywheelRPSSupplier = supplier;
    }

    public void setHoodSetpoint(HoodSetpoint setpoint) {
        Rotation2d target = setpoint == HoodSetpoint.DYNAMIC ? hoodAngleSupplier.get() : setpoint.getAngle();
        setHoodAngle(target);
    }

    public void setFlywheelSetpoint(FlywheelSetpoint setpoint) {
        double target = setpoint == FlywheelSetpoint.DYNAMIC ? flywheelRPSSupplier.get() : setpoint.getRps();
        setShotVelocity(target);
    }

    // Direct setters for mechanism control (now private, used internally by state handlers)
    public void setHoodAngle(Rotation2d angle) {
        double clampedAngle = MathUtil.clamp(angle.getRotations(), config.hoodMinAngleRotations, config.hoodMaxAngleRotations); 

        hoodSetpointRotations = clampedAngle;
        Logger.recordOutput("Shooter/angleSetpointRotations", clampedAngle);
        Logger.recordOutput("Shooter/rawSetpointRotations", clampedAngle);

        shooterIO.setAngle(clampedAngle);
    }

    public void setShotVelocity(double velocityRotationsPerSec) {
        flywheelSetpointRPS = velocityRotationsPerSec;
        Logger.recordOutput("Shooter/shotVelocitySetpointRotationsPerSec", velocityRotationsPerSec);
        shooterIO.setShotVelocity(velocityRotationsPerSec);
    }

    // Mechanism getters
    public double getHoodAngleRotations() {
        return shooterInputs.hoodAngleRotations;
    }

    public double getFlywheelVelocityRotationsPerSec() {
        return shooterInputs.flywheelVelocityRotationsPerSec;
    }

    /**
     * Calculate ball backspin from differential roller surface speeds.
     * Flywheel (bottom) moving faster than back roller (top) creates backspin.
     */
    public double calculateBackSpinRPM(double flywheelVelocityRotationsPerSec) {
        double flywheelSurfaceVel = flywheelVelocityRotationsPerSec * 2 * Math.PI * config.flywheelRadiusMeters;
        double backRollerSurfaceVel = flywheelVelocityRotationsPerSec * config.backRollerGearRatio
            * 2 * Math.PI * config.backRollerRadiusMeters;

        double deltaV = flywheelSurfaceVel - backRollerSurfaceVel;
        double ballRadiusMeters = FieldConstants.fuelDiameter / 2.0;
        double spinRadPerSec = deltaV / ballRadiusMeters;

        return spinRadPerSec * 60.0 / (2 * Math.PI);
    }

    public double calculateShotExitVelocityMetersPerSec(double flywheelVelocityRotationsPerSec) {
        double flywheelSurfaceVel = flywheelVelocityRotationsPerSec * 2 * Math.PI * config.flywheelRadiusMeters;
        double backRollerSurfaceVel = flywheelVelocityRotationsPerSec * config.backRollerGearRatio
            * 2 * Math.PI * config.backRollerRadiusMeters;
        return (flywheelSurfaceVel + backRollerSurfaceVel) / 2.0;
    }

    public double getShotExitVelocityMetersPerSec() {
        return calculateShotExitVelocityMetersPerSec(shooterInputs.flywheelVelocityRotationsPerSec);
    }

    // Setpoint check methods
    @AutoLogOutput(key = "Shooter/isHoodAtSetpoint")
    public boolean isHoodAtSetpoint() {
        return Math.abs(shooterInputs.hoodAngleRotations - hoodSetpointRotations) < config.hoodAngleToleranceRotations;
    }

    @AutoLogOutput(key = "Shooter/isFlywheelAtSetpoint")
    public boolean isFlywheelAtSetpoint() {
        return Math.abs(shooterInputs.flywheelVelocityRotationsPerSec - flywheelSetpointRPS) < config.flywheelVelocityToleranceRPS;
    }

    // Config getters
    public InterpolatingMatrixTreeMap<Double, N2, N1> getLerpTable() {
        return config.getLerpTable();
    }

    public Pose3d getShooterRelativePose() {
        return config.getShooterPose3d();
    }

    public double getMinShotDistFromShooterMeters() {
        return config.minShotDistFromShooterMeters;
    }

    public double getMaxShotDistFromShooterMeters() {
        return config.maxShotDistFromShooterMeters;
    }

    public double getLatencyCompensationSeconds() {
        return config.latencyCompensationSeconds;
    }
}
