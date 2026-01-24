package frc.robot.configs;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;

public class VisionConfig {
    public static class CameraConfig {
        public String name;
        public double x;
        public double y;
        public double z;
        public double rollDeg;
        public double pitchDeg;
        public double yawDeg;

        public Transform3d getRobotToCamera() {
            return new Transform3d(
                x,
                y,
                z,
                new Rotation3d(
                    Math.toRadians(rollDeg),
                    Math.toRadians(pitchDeg),
                    Math.toRadians(yawDeg)));
        }
    }

    public List<CameraConfig> cameras;

    public int disabledImuMode;
    public int enabledImuMode;

    public double maxAmbiguity;
    public double maxZError;

    public double linearStdDevBaseline;
    public double angularStdDevBaseline;
    public double linearStdDevMegatag1Factor;
    public double angularStdDevMegatag1Factor;
    public double linearStdDevMegatag2Factor;
    public double angularStdDevMegatag2Factor;

    public double maxRotationRateMegatag1DegreesPerSecond;
    public double maxRotationRateMegatag2DegreesPerSecond;

    public double[] cameraStdDevFactors;
}
