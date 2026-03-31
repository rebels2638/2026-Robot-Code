package frc.robot.sim;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.configs.SwerveConfig;
import frc.robot.configs.SwerveDrivetrainConfig;
import frc.robot.configs.SwerveModuleGeneralConfig;
import frc.robot.constants.Constants;
import frc.robot.lib.util.ConfigLoader;

/** Shared Maple simulation world for drivetrain, odometry, and vision. */
public final class MapleSimManager {
    private static final double ROBOT_MASS_KG = 52.0;
    private static final double BUMPER_LENGTH_METERS = 0.86;
    private static final double BUMPER_WIDTH_METERS = 0.86;
    private static final double DRIVE_FRICTION_VOLTS = 0.1;
    private static final double STEER_FRICTION_VOLTS = 0.2;
    private static final double STEER_MOI_KG_M2 = 0.03;

    private int hubHitCount = 0;

    private static MapleSimManager instance;

    public static MapleSimManager getInstance() {
        if (instance == null) {
            instance = new MapleSimManager();
        }
        return instance;
    }

    private final SimulatedArena arena;
    private final SwerveDriveSimulation driveSimulation;

    private MapleSimManager() {
        SwerveConfig swerveConfig = ConfigLoader.load(
            "swerve",
            ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.SWERVE),
            SwerveConfig.class
        );
        driveSimulation = new SwerveDriveSimulation(
            createDriveTrainSimulationConfig(swerveConfig.drivetrain, swerveConfig.moduleGeneral),
            new Pose2d()
        );

        arena = SimulatedArena.getInstance();
        arena.addDriveTrainSimulation(driveSimulation);
        arena.resetFieldForAuto();
    }

    private static DriveTrainSimulationConfig createDriveTrainSimulationConfig(
        SwerveDrivetrainConfig drivetrainConfig,
        SwerveModuleGeneralConfig moduleGeneralConfig
    ) {
        Translation2d[] moduleTranslations = new Translation2d[] {
            drivetrainConfig.getFrontLeftPositionMeters(),
            drivetrainConfig.getFrontRightPositionMeters(),
            drivetrainConfig.getBackLeftPositionMeters(),
            drivetrainConfig.getBackRightPositionMeters()
        };

        SwerveModuleSimulationConfig moduleSimulationConfig = new SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60Foc(1),
            DCMotor.getKrakenX60Foc(1),
            moduleGeneralConfig.driveMotorToOutputShaftRatio,
            moduleGeneralConfig.steerMotorToOutputShaftRatio,
            Volts.of(DRIVE_FRICTION_VOLTS),
            Volts.of(STEER_FRICTION_VOLTS),
            Meters.of(moduleGeneralConfig.driveWheelRadiusMeters),
            KilogramSquareMeters.of(STEER_MOI_KG_M2),
            COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof
        );

        return DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
            .withBumperSize(Meters.of(BUMPER_LENGTH_METERS), Meters.of(BUMPER_WIDTH_METERS))
            .withCustomModuleTranslations(moduleTranslations)
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(moduleSimulationConfig);
    }

    public SimulatedArena getArena() {
        return arena;
    }

    public SwerveDriveSimulation getDriveSimulation() {
        return driveSimulation;
    }

    public SwerveModuleSimulation getModuleSimulation(int moduleIndex) {
        return driveSimulation.getModules()[moduleIndex];
    }

    public GyroSimulation getGyroSimulation() {
        return driveSimulation.getGyroSimulation();
    }

    public Pose2d getActualPose() {
        return driveSimulation.getSimulatedDriveTrainPose();
    }

    public void resetRobotPose(Pose2d pose) {
        driveSimulation.setSimulationWorldPose(pose);
        driveSimulation.setRobotSpeeds(new ChassisSpeeds());
        driveSimulation.getGyroSimulation().setRotation(pose.getRotation());
    }

    public void resetFieldForAuto() {
        arena.resetFieldForAuto();
    }

    /** Called by projectile hit callbacks to track Hub scoring. */
    public void incrementHubHitCount() {
        hubHitCount++;
        Logger.recordOutput("MapleSim/HubHitCount", hubHitCount);
        System.out.println("[MapleSim] FUEL hits HUB! Total: " + hubHitCount);
    }

    public double[] getOdometryTimestampsSeconds() {
        int subTicks = SimulatedArena.getSimulationSubTicksIn1Period();
        double dtSeconds = SimulatedArena.getSimulationDt().in(Seconds);
        double nowSeconds = Timer.getTimestamp();
        double[] timestampsSeconds = new double[subTicks];
        double periodStartSeconds = nowSeconds - (dtSeconds * subTicks);

        for (int i = 0; i < subTicks; i++) {
            timestampsSeconds[i] = periodStartSeconds + (dtSeconds * (i + 1));
        }

        return timestampsSeconds;
    }

    public void simulationPeriodic() {
        arena.simulationPeriodic();
        Logger.recordOutput("MapleSim/ActualRobotPose", getActualPose());
        Logger.recordOutput(
            "MapleSim/ActualRobotSpeedsRobotRelative",
            driveSimulation.getDriveTrainSimulatedChassisSpeedsRobotRelative()
        );
        Logger.recordOutput("FieldSimulation/FuelPoses", arena.getGamePiecesArrayByType("Fuel"));
        Logger.recordOutput("MapleSim/HubHitCount", hubHitCount);
    }
}
