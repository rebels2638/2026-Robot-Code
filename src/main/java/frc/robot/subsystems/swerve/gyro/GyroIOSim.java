package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;

import org.ironmaple.simulation.drivesims.GyroSimulation;

/**
 * Gyro IO implementation backed by maple-sim's GyroSimulation.
 * Provides simulated gyro data (yaw, yaw velocity) with realistic noise/drift
 * modeled by maple-sim.
 */
public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = true;

        inputs.gyroOrientation = new Rotation3d(
                0.0,
                0.0,
                gyroSimulation.getGyroReading().getRadians());
        inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(
                edu.wpi.first.units.Units.RadiansPerSecond);

        Rotation2d[] cachedYawPositions = gyroSimulation.getCachedGyroReadings();
        inputs.odometryYawPositions = cachedYawPositions;

        double currentTime = Timer.getFPGATimestamp();
        double dt = 0.02 / cachedYawPositions.length;

        inputs.odometryTimestampsSeconds = new double[cachedYawPositions.length];
        for (int i = 0; i < cachedYawPositions.length; i++) {
            inputs.odometryTimestampsSeconds[i] = currentTime - (cachedYawPositions.length - 1 - i) * dt;
        }
    }

    @Override
    public void resetGyro(Rotation2d yaw) {
        // pose estimator handles the offset
    }
}
