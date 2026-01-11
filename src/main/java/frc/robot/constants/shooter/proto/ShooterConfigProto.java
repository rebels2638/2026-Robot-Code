package frc.robot.constants.shooter.proto;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import frc.robot.constants.shooter.ShooterConfigBase;

public class ShooterConfigProto extends ShooterConfigBase {

    public static ShooterConfigProto instance = null;
    public static ShooterConfigProto getInstance() {
        if (instance == null) {
            instance = new ShooterConfigProto();
        }
        return instance;
    }

    private ShooterConfigProto() {}

    @Override
    public InterpolatingMatrixTreeMap<Double, N2, N1> getLerpTable() {
        InterpolatingMatrixTreeMap<Double, N2, N1> table = new InterpolatingMatrixTreeMap<>();
        table.put(0.2, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(0.4, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(0.6, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(0.8, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(1.0, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(1.2, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(1.4, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(1.6, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(1.8, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        return table;
    }

    @Override
    public double getMinShotDistFromShooterMeters() {
        return 0.2;
    }

    @Override
    public double getMaxShotDistFromShooterMeters() {
        return 1.8;
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
        return 0.1;
    }

    @Override
    public double getHoodKV() {
        return 0.12;
    }

    @Override
    public double getHoodKA() {
        return 0.01;
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
        return 2.5;
    }

    @Override
    public double getHoodMotorToOutputShaftRatio() {
        return 100.0;
    }

    @Override
    public double getHoodStartingAngleRotations() {
        return 90.0/360.0;
    }

    @Override
    public double getHoodMinAngleRotations() {
        return -0.5 + 90.0/360.0;
    }

    @Override
    public double getHoodMaxAngleRotations() {
        return 0.5 + 90.0/360.0;
    }

    // Turret motor config (mirrors hood defaults; update for real hardware as needed)
    @Override
    public int getTurretCanId() {
        // TODO: Update to real turret CAN ID
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
        return 0.0;
    }

    @Override
    public double getTurretMinAngleDeg() {
        // +/- 180 degrees by default
        return -180.0;
    }

    @Override
    public double getTurretMaxAngleDeg() {
        return 180.0;
    }

    @Override
    public double getTurretMaxVelocityDegPerSec() {
        // Default turret cruise velocity (deg/s), tune as needed
        return 180.0;
    }

    @Override
    public double getTurretMaxAccelerationDegPerSec2() {
        // Default turret acceleration (deg/s^2), tune as needed
        return 360.0;
    }

    // Flywheel motor config
    @Override
    public int getFlywheelCanId() {
        return 21;
    }

    @Override
    public boolean getIsFlywheelInverted() {
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
        return 0.1;
    }

    @Override
    public double getFlywheelKV() {
        return 0.002;
    }

    @Override
    public double getFlywheelKA() {
        return 0.0005;
    }

    @Override
    public double getFlywheelKP() {
        return 0.08;
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

    // Feeder motor config
    @Override
    public int getFeederCanId() {
        return 22;
    }

    @Override
    public boolean getIsFeederInverted() {
        return false;
    }

    @Override
    public double getFeederSupplyCurrentLimit() {
        return 30.0;
    }

    @Override
    public double getFeederSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getFeederSupplyCurrentLimitLowerLimit() {
        return 25.0;
    }

    @Override
    public double getFeederStatorCurrentLimit() {
        return 40.0;
    }

    @Override
    public double getFeederPeakForwardTorqueCurrent() {
        return 40.0;
    }

    @Override
    public double getFeederPeakReverseTorqueCurrent() {
        return -40.0;
    }

    @Override
    public double getFeederKS() {
        return 0.1;
    }

    @Override
    public double getFeederKV() {
        return 0.002;
    }

    @Override
    public double getFeederKA() {
        return 0.0005;
    }

    @Override
    public double getFeederKP() {
        return 0.08;
    }

    @Override
    public double getFeederKI() {
        return 0.0;
    }

    @Override
    public double getFeederKD() {
        return 0.0;
    }

    @Override
    public double getFeederMotorToOutputShaftRatio() {
        return 5.0;
    }

    // Indexer motor config
    @Override
    public int getIndexerCanId() {
        return 23;
    }

    @Override
    public boolean getIsIndexerInverted() {
        return false;
    }

    @Override
    public double getIndexerSupplyCurrentLimit() {
        return 30.0;
    }

    @Override
    public double getIndexerSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getIndexerSupplyCurrentLimitLowerLimit() {
        return 25.0;
    }

    @Override
    public double getIndexerStatorCurrentLimit() {
        return 40.0;
    }

    @Override
    public double getIndexerPeakForwardTorqueCurrent() {
        return 40.0;
    }

    @Override
    public double getIndexerPeakReverseTorqueCurrent() {
        return -40.0;
    }

    @Override
    public double getIndexerKS() {
        return 0.1;
    }

    @Override
    public double getIndexerKV() {
        return 0.002;
    }

    @Override
    public double getIndexerKA() {
        return 0.0005;
    }

    @Override
    public double getIndexerKP() {
        return 0.08;
    }

    @Override
    public double getIndexerKI() {
        return 0.0;
    }

    @Override
    public double getIndexerKD() {
        return 0.0;
    }

    @Override
    public double getIndexerMotorToOutputShaftRatio() {
        return 5.0;
    }

    @Override
    public double getHoodAngleToleranceRotations() {
        return 5.0 / 360.0;
    }

    @Override
    public double getTurretAngleToleranceRotations() {
        // Match hood tolerance by default
        return getHoodAngleToleranceRotations();
    }

    @Override
    public double getFlywheelVelocityToleranceRPS() {
        return 3.0;
    }

    @Override
    public double getFeederVelocityToleranceRPS() {
        return 5.0;
    }

    @Override
    public double getIndexerVelocityToleranceRPS() {
        return 5.0;
    }
}
