package frc.robot.constants.swerve.drivetrainConfigs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.swerve.moduleConfigs.sim.SwerveModuleGeneralConfigSim;

public class SwerveDrivetrainConfigSim extends SwerveDrivetrainConfigBase {

    public static SwerveDrivetrainConfigSim instance = null;

    public static SwerveDrivetrainConfigSim getInstance() {
        if (instance == null) {
            instance = new SwerveDrivetrainConfigSim();
        }
        return instance;
    }

    private SwerveDrivetrainConfigSim() {}

    @Override
    public double getMaxTranslationalVelocityMetersPerSec() {
        return getMaxModuleVelocity();
    }

    @Override
    public double getMaxTranslationalAccelerationMetersPerSecSec() {
        return 14;
    }

    @Override
    public double getMaxAngularVelocityRadiansPerSec() {
        return getMaxModuleVelocity() / Math.hypot(getFrontLeftPositionMeters().getX(), getFrontLeftPositionMeters().getY()); 
    }

    @Override
    public double getMaxAngularAccelerationRadiansPerSecSec() {
        return Math.toRadians(2000);
    }

    @Override
    public double getMaxModuleVelocity() {
        return 4.5;
    }

    @Override
    public Translation2d getFrontLeftPositionMeters() {
        return new Translation2d(0.23, 0.23);
    }

    @Override
    public Translation2d getFrontRightPositionMeters() {
        return new Translation2d(0.23, -0.23);
    }

    @Override
    public Translation2d getBackLeftPositionMeters() {
        return new Translation2d(-0.23, 0.23);
    }

    @Override
    public Translation2d getBackRightPositionMeters() {
        return new Translation2d(-0.23, -0.23);
    }

    @Override
    public double getRotationCompensationCoefficient() {
        return 0.0;
    }

    @Override
    public PIDController getTranslationController() {
        PIDController controller = new PIDController(4.0, 0.0, 0.0);
        controller.setTolerance(getTranslationToleranceMeters(), getTranslationVelocityToleranceMeters());
        return controller;
    }

    @Override
    public PIDController getRotationController() {
        PIDController controller = new PIDController(12.0, 0.0, 1.1);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(Math.toRadians(getRotationToleranceDeg()), Math.toRadians(getRotationVelocityToleranceDegPerSec()));

        return controller;
    }

    @Override
    public double getTranslationToleranceMeters() {
        return 0.05;
    }

    @Override
    public double getTranslationVelocityToleranceMeters() {
        return 0.15;
    }

    @Override
    public double getRotationToleranceDeg() {
        return 3.5;
    }

    @Override
    public double getRotationVelocityToleranceDegPerSec() {
        return 5.0;
    }

    @Override
    public double getRangedRotationKP() {
        return 12.0;
    }

    @Override
    public double getRangedRotationKI() {
        return 0.0;
    }

    @Override
    public double getRangedRotationKD() {
        return 1.1;
    }

    @Override
    public double getRangedRotationToleranceDeg() {
        return 10.0;
    }

    @Override
    public double getFollowPathTranslationKP() {
        return 6.3;
    }

    @Override
    public double getFollowPathTranslationKI() {
        return 0.0;
    }

    @Override
    public double getFollowPathTranslationKD() {
        return 0.0;
    }

    @Override
    public double getFollowPathRotationKP() {
        return 12.0;
    }

    @Override
    public double getFollowPathRotationKI() {
        return 0.0;
    }

    @Override
    public double getFollowPathRotationKD() {
        return 1.1;
    }

    @Override
    public double getFollowPathCrossTrackKP() {
        return 6; // 6
    }

    @Override
    public double getFollowPathCrossTrackKI() {
        return 0;  // .1
    }

    @Override
    public double getFollowPathCrossTrackKD() {
        return 0; // .2
    }
}