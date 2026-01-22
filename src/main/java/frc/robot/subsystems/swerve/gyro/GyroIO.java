package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean isConnected = false;
        public Rotation3d gyroOrientation = new Rotation3d();
        public double yawVelocityRadPerSec = 0.0;

        public double[] odometryTimestampsSeconds = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {}
    public default void resetGyro(Rotation2d yaw) {};
}