package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.sim.MapleSimManager;

public class GyroIOSim implements GyroIO {
    private final MapleSimManager mapleSim = MapleSimManager.getInstance();

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        Rotation2d gyroReading = mapleSim.getGyroSimulation().getGyroReading();

        inputs.isConnected = true;
        inputs.gyroOrientation = new Rotation3d(0.0, 0.0, gyroReading.getRadians());
        inputs.yawVelocityRadPerSec =
            mapleSim.getGyroSimulation().getMeasuredAngularVelocity().in(RadiansPerSecond);
        inputs.odometryTimestampsSeconds = mapleSim.getOdometryTimestampsSeconds();
        inputs.odometryYawPositions = mapleSim.getGyroSimulation().getCachedGyroReadings().clone();
    }

    @Override
    public void resetGyro(Rotation2d yaw) {
        mapleSim.getGyroSimulation().setRotation(yaw);
    }
}
