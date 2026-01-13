package frc.robot.constants.shooter.sim;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import frc.robot.constants.shooter.ShooterConfigBase;

public class ShooterConfigSim extends ShooterConfigBase {

    public static ShooterConfigSim instance = null;
    public static ShooterConfigSim getInstance() {
        if (instance == null) {
            instance = new ShooterConfigSim();
        }
        return instance;
    }

    private ShooterConfigSim() {}
    
    @Override
    // angle rotations, flywheel velocity in rotations per second
    // distance is HORIZONTAL distance from shooter itself to target in meters
    public InterpolatingMatrixTreeMap<Double, N2, N1> getLerpTable() {
        // Calculated using idealized projectile motion physics with:
        // - Flywheel radius: 0.0508m (2 inches)
        // - Shooter height: 0.46m above ground (from getShooterPose3d)
        // - Target height: 0.46m above ground (from Constants.FieldConstants.kSHOOTER_TARGET)
        // - Exit velocity = flywheelRPS * π * radius (accounting for /2 physical factor)
        // - Hood angle relative to horizontal
        // - Robot assumed to be standing still
        // - Since shooter and target are at same height, optimal angle is 45° (0.125 rotations)
        // - Range formula for same height at 45°: R = v² / g where g = 9.81 m/s²
        // - Required exit velocity: v = √(R × g)
        // - Required RPS: RPS = v / (π × 0.0508) [accounting for /2 factor in velocity calc]
        InterpolatingMatrixTreeMap<Double, N2, N1> table = new InterpolatingMatrixTreeMap<>();

        // Distance 0.5m: v = √(0.5 × 9.81) = 2.215 m/s, RPS = 2.215 / (π × 0.0508) = 13.88
        table.put(0.5, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.125, // hood angle = 45.00° (optimal for same-height trajectory)
            13.88  // flywheel velocity in RPS
        }));

        // Distance 1.0m: v = √(1.0 × 9.81) = 3.132 m/s, RPS = 3.132 / (π × 0.0508) = 19.62
        table.put(1.0, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.125, // hood angle = 45.00°
            19.62  // flywheel velocity in RPS
        }));

        // Distance 1.5m: v = √(1.5 × 9.81) = 3.836 m/s, RPS = 3.836 / (π × 0.0508) = 24.03
        table.put(1.5, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.125, // hood angle = 45.00°
            24.03  // flywheel velocity in RPS
        }));

        // Distance 2.0m: v = √(2.0 × 9.81) = 4.429 m/s, RPS = 4.429 / (π × 0.0508) = 27.75
        table.put(2.0, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.125, // hood angle = 45.00°
            27.75  // flywheel velocity in RPS
        }));

        // Distance 2.5m: v = √(2.5 × 9.81) = 4.952 m/s, RPS = 4.952 / (π × 0.0508) = 31.03
        table.put(2.5, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.125, // hood angle = 45.00°
            31.03  // flywheel velocity in RPS
        }));

        // Distance 3.0m: v = √(3.0 × 9.81) = 5.425 m/s, RPS = 5.425 / (π × 0.0508) = 33.99
        table.put(3.0, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.125, // hood angle = 45.00°
            33.99  // flywheel velocity in RPS
        }));

        // Distance 3.5m: v = √(3.5 × 9.81) = 5.860 m/s, RPS = 5.860 / (π × 0.0508) = 36.71
        table.put(3.5, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.125, // hood angle = 45.00°
            36.71  // flywheel velocity in RPS
        }));

        // Distance 4.0m: v = √(4.0 × 9.81) = 6.264 m/s, RPS = 6.264 / (π × 0.0508) = 39.25
        table.put(4.0, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.125, // hood angle = 45.00°
            39.25  // flywheel velocity in RPS
        }));

        // Distance 4.5m: v = √(4.5 × 9.81) = 6.644 m/s, RPS = 6.644 / (π × 0.0508) = 41.63
        table.put(4.5, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.125, // hood angle = 45.00°
            41.63  // flywheel velocity in RPS
        }));

        return table;
    }

    @Override
    public double getMinShotDistFromShooterMeters() {
        return 0.5;
    }

    @Override
    public double getMaxShotDistFromShooterMeters() {
        return 4.5;
    }

    @Override
    public double getLatencyCompensationSeconds() {
        return 0.0;
    }

    @Override
    public String getCanBusName() {
        return "rio";
    }

    // Hood motor config
    @Override
    public int getHoodCanId() {
        return 20;
    }

    @Override
    public boolean getIsHoodInverted() {
        return false;
    }

    @Override
    public double getHoodSupplyCurrentLimit() {
        return 30.0;
    }

    @Override
    public double getHoodSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getHoodSupplyCurrentLimitLowerLimit() {
        return 25.0;
    }

    @Override
    public double getHoodStatorCurrentLimit() {
        return 40.0;
    }

    @Override
    public double getHoodPeakForwardTorqueCurrent() {
        return 40.0;
    }

    @Override
    public double getHoodPeakReverseTorqueCurrent() {
        return -40.0;
    }

    @Override
    public double getHoodKS() {
        return 0;
    }

    @Override
    public double getHoodKV() {
        return 0;
    }

    @Override
    public double getHoodKA() {
        return 0.0;
    }

    @Override
    public double getHoodKP() {
        return 25.0;
    }

    @Override
    public double getHoodKI() {
        return 0.0;
    }

    @Override
    public double getHoodKD() {
        return 0.0;
    }

    @Override
    public double getHoodMotorToOutputShaftRatio() {
        return 100.0;
    }

    // shot exit angle above horizontal
    @Override
    public double getHoodStartingAngleRotations() {
        return 54.0/360.0;
    }

    @Override
    public double getHoodMinAngleRotations() {
        return -80/360.0;
    }

    @Override
    public double getHoodMaxAngleRotations() {
        return 54.0/360.0; // 150
    }

    // Turret motor config (mirrors hood defaults; update for real hardware as needed)
    @Override
    public int getTurretCanId() {
        // Sim-only; does not need a real CAN ID but keep distinct from hood
        return 19;
    }

    @Override
    public boolean getIsTurretInverted() {
        return getIsHoodInverted();
    }

    @Override
    public double getTurretSupplyCurrentLimit() {
        return getHoodSupplyCurrentLimit();
    }

    @Override
    public double getTurretSupplyCurrentLimitLowerTime() {
        return getHoodSupplyCurrentLimitLowerTime();
    }

    @Override
    public double getTurretSupplyCurrentLimitLowerLimit() {
        return getHoodSupplyCurrentLimitLowerLimit();
    }

    @Override
    public double getTurretStatorCurrentLimit() {
        return getHoodStatorCurrentLimit();
    }

    @Override
    public double getTurretPeakForwardTorqueCurrent() {
        return getHoodPeakForwardTorqueCurrent();
    }

    @Override
    public double getTurretPeakReverseTorqueCurrent() {
        return getHoodPeakReverseTorqueCurrent();
    }

    @Override
    public double getTurretKS() {
        return getHoodKS();
    }

    @Override
    public double getTurretKV() {
        return getHoodKV();
    }

    @Override
    public double getTurretKA() {
        return getHoodKA();
    }

    @Override
    public double getTurretKP() {
        return getHoodKP();
    }

    @Override
    public double getTurretKI() {
        return getHoodKI();
    }

    @Override
    public double getTurretKD() {
        return getHoodKD();
    }

    @Override
    public double getTurretMotorToOutputShaftRatio() {
        return getHoodMotorToOutputShaftRatio();
    }

    @Override
    public double getTurretStartingAngleDeg() {
        // Default turret starting angle: pointing straight ahead
        return 180.0;
    }

    @Override
    public double getTurretMinAngleDeg() {
        // +/- 180 degrees by default
        return 80.0;
    }

    @Override
    public double getTurretMaxAngleDeg() {
        return 280.0;
    }

    @Override
    public double getTurretMaxVelocityDegPerSec() {
        // Default turret cruise velocity (deg/s), tune as needed
        return 900.0;
    }

    @Override
    public double getTurretMaxAccelerationDegPerSec2() {
        // Default turret acceleration (deg/s^2), tune as needed
        return 3000.0;
    }

    // Flywheel motor config - 2 motors with 1:1 gearing (leader/follower)
    @Override
    public int getFlywheelCanId() {
        return 21;
    }

    @Override
    public int getFlywheelFollowerCanId() {
        return 22;
    }

    @Override
    public boolean getIsFlywheelInverted() {
        return false;
    }

    @Override
    public boolean getIsFlywheelFollowerOppositeDirection() {
        return false;
    }

    @Override
    public double getFlywheelSupplyCurrentLimit() {
        return 60.0;
    }

    @Override
    public double getFlywheelSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getFlywheelSupplyCurrentLimitLowerLimit() {
        return 50.0;
    }

    @Override
    public double getFlywheelStatorCurrentLimit() {
        return 80.0;
    }

    @Override
    public double getFlywheelPeakForwardTorqueCurrent() {
        return 80.0;
    }

    @Override
    public double getFlywheelPeakReverseTorqueCurrent() {
        return -80.0;
    }

    @Override
    public double getFlywheelKS() {
        return 0;
    }

    @Override
    public double getFlywheelKV() {
        return 0.03;
    }

    @Override
    public double getFlywheelKA() {
        return 0;
    }

    @Override
    public double getFlywheelKP() {
        return 0.02;
    }

    @Override
    public double getFlywheelKI() {
        return 0.0;
    }

    @Override
    public double getFlywheelKD() {
        return 0.0;
    }

    @Override
    public double getFlywheelMotorToOutputShaftRatio() {
        return 1.0;
    }

    @Override
    public double getFlywheelRadiusMeters() {
        return 0.0508; // 2 inches in meters
    }

    @Override
    public Pose3d getShooterPose3d() {
        // Shooter position relative to robot center
        // X: forward from robot center (meters)
        // Y: left/right offset (meters, positive = left)
        // Z: height above ground (meters)
        // Yaw: 0 means turret angle 0 points forward relative to robot
        return new Pose3d(
            new Translation3d(-0.11, 0.0, 0.46),
            new Rotation3d(0.0, 0.0, 0.0)
        );
    }

    @Override
    public double getHoodAngleToleranceRotations() {
        return 2.0 / 360.0;
    }

    @Override
    public double getTurretAngleToleranceRotations() {
        // Match hood tolerance by default
        return 2.0 / 360.0;
    }

    @Override
    public double getFlywheelVelocityToleranceRPS() {
        return 3.0;
    }
}
