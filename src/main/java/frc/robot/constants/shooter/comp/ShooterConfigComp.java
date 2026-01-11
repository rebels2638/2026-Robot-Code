package frc.robot.constants.shooter.comp;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import frc.robot.constants.shooter.ShooterConfigBase;

public class ShooterConfigComp extends ShooterConfigBase {

    public static ShooterConfigComp instance = null;
    public static ShooterConfigComp getInstance() {
        if (instance == null) {
            instance = new ShooterConfigComp();
        }
        return instance;
    }

    private ShooterConfigComp() {}

    @Override
    // angle rotations, flywheel velocity in rotations per second
    // distance is distance from shooter itself to target in meters
    public InterpolatingMatrixTreeMap<Double, N2, N1> getLerpTable() {
        // Calculated using idealized projectile motion physics with:
        // - Flywheel radius: 0.0508m (2 inches)
        // - Shooter height: 0.46m above ground (from getShooterPose3d)
        // - Target height: 0.46m above ground (from Constants.FieldConstants.kSHOOTER_TARGET)
        // - Exit velocity = flywheelRPS * π * radius (from Shooter.calculateShotExitVelocityMetersPerSec)
        // - Hood angle relative to horizontal
        // - Robot assumed to be standing still
        // - Since shooter and target are at same height, optimal angle is 45° (0.125 rotations)
        // - Range formula for same height: R = v²/g where g = 9.81 m/s²
        // - Required velocity: v = √(R × g)
        // - Required RPS: RPS = v / (π × 0.0508)
        InterpolatingMatrixTreeMap<Double, N2, N1> table = new InterpolatingMatrixTreeMap<>();
        
        table.put(0.48, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            45.0/360.0, // hood angle = 45.00° (optimal for flat trajectory)
            12 // flywheel velocity in RPS (exit velocity = 1.88 m/s)
        }));

        table.put(0.71, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            45.0/360.0, // hood angle = 45.00° (optimal for flat trajectory)
            15.0 // flywheel velocity in RPS (exit velocity = 1.88 m/s)
        }));

        table.put(1.08 , new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            40.0/360.0, // hood angle = 45.00° (optimal for flat trajectory)
            21.0 // flywheel velocity in RPS (exit velocity = 1.88 m/s)
        }));

        table.put(1.32 , new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            40.0/360.0, // hood angle = 45.00° (optimal for flat trajectory)
            23.0 // flywheel velocity in RPS (exit velocity = 1.88 m/s)
        }));

        table.put(2.132, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            40.0/360, // hood angle = 45.00°
            30.0 // flywheel velocity in RPS (exit velocity = 3.65 m/s)
        }));
        
        
        return table;
    }

    @Override
    public double getMinShotDistFromShooterMeters() {
        return 0.48;
    }

    @Override
    public double getMaxShotDistFromShooterMeters() {
        return 2.132;
    }

    @Override
    public double getLatencyCompensationSeconds() {
        return 0.3;
    }

    @Override
    public String getCanBusName() {
        return "drivetrain";
    }

    // Hood motor config
    @Override
    public int getHoodCanId() {
        return 19;
    }

    @Override
    public boolean getIsHoodInverted() {
        return true;
    }

    @Override
    public double getHoodSupplyCurrentLimit() {
        return 80.0;
    }

    @Override
    public double getHoodSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getHoodSupplyCurrentLimitLowerLimit() {
        return 80.0;
    }

    @Override
    public double getHoodStatorCurrentLimit() {
        return 80.0;
    }

    @Override
    public double getHoodPeakForwardTorqueCurrent() {
        return 80.0;
    }

    @Override
    public double getHoodPeakReverseTorqueCurrent() {
        return -80.0;
    }

    @Override
    public double getHoodKS() {
        return 0.22;
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
        return 50; // 300
    }

    @Override
    public double getHoodKI() {
        return 20; // 10
    }

    @Override
    public double getHoodKD() {
        return 1.0; // 12
    }

    @Override
    public double getHoodMotorToOutputShaftRatio() {
        return 84.0/36.0; 
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
        return 33;
    }

    @Override
    public boolean getIsTurretInverted() {
        return true;
    }

    @Override
    public double getTurretSupplyCurrentLimit() {
        return 90.0;
    }

    @Override
    public double getTurretSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getTurretSupplyCurrentLimitLowerLimit() {
        return 90.0;
    }

    @Override
    public double getTurretStatorCurrentLimit() {
        return 90.0;
    }

    @Override
    public double getTurretPeakForwardTorqueCurrent() {
        return 90.0;
    }

    @Override
    public double getTurretPeakReverseTorqueCurrent() {
        return -90.0;
    }

    @Override
    public double getTurretKS() {
        return 0.22;
    }

    @Override
    public double getTurretKV() {
        return 0.0;
    }

    @Override
    public double getTurretKA() {
        return 0.0;
    }

    @Override
    public double getTurretKP() {
        return 75;
    }

    @Override
    public double getTurretKI() {
        return 0.0;
    }

    @Override
    public double getTurretKD() {
        return 0.0;
    }

    @Override
    public double getTurretMotorToOutputShaftRatio() {
        return 120.0/18.0;
    }

    @Override
    public double getTurretStartingAngleDeg() {
        // Default turret starting angle: pointing straight ahead
        return 180.0;
    }

    @Override
    public double getTurretMinAngleDeg() {
        // +/- 180 degrees by default
        return 90.0;
    }

    @Override
    public double getTurretMaxAngleDeg() {
        return 270.0;
    }

    @Override
    public double getTurretMaxVelocityDegPerSec() {
        // Default turret cruise velocity (deg/s), tune as needed
        return 720.0;
    }

    @Override
    public double getTurretMaxAccelerationDegPerSec2() {
        // Default turret acceleration (deg/s^2), tune as needed
        return 3000.0;
    }

    // Flywheel motor config
    @Override
    public int getFlywheelCanId() {
        return 16;
    }

    @Override
    public boolean getIsFlywheelInverted() {
        return false;
    }

    @Override
    public double getFlywheelSupplyCurrentLimit() {
        return 15.0;
    }

    @Override
    public double getFlywheelSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getFlywheelSupplyCurrentLimitLowerLimit() {
        return 15.0;
    }

    @Override
    public double getFlywheelStatorCurrentLimit() {
        return 15.0;
    }

    @Override
    public double getFlywheelPeakForwardTorqueCurrent() {
        return 15.0;
    }

    @Override
    public double getFlywheelPeakReverseTorqueCurrent() {
        return -15.0;
    }

    @Override
    public double getFlywheelKS() {
        return 0.1;
    }

    @Override
    public double getFlywheelKV() {
        return 0.188;
    }

    @Override
    public double getFlywheelKA() {
        return 0.0;
    }

    @Override
    public double getFlywheelKP() {
        return 0.5;
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
        return 18.0/12.0; // Direct drive
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
            new Translation3d(-0.19, 0.0, 0.46),
            new Rotation3d(0.0, 0.0, 0.0)
        );
    }

    // Feeder motor config
    @Override
    public int getFeederCanId() {
        return 35;
    }

    @Override
    public boolean getIsFeederInverted() {
        return false;
    }

    @Override
    public double getFeederSupplyCurrentLimit() {
        return 90.0;
    }

    @Override
    public double getFeederSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getFeederSupplyCurrentLimitLowerLimit() {
        return 90.0;
    }

    @Override
    public double getFeederStatorCurrentLimit() {
        return 90.0;
    }

    @Override
    public double getFeederPeakForwardTorqueCurrent() {
        return 90.0;
    }

    @Override
    public double getFeederPeakReverseTorqueCurrent() {
        return -90.0;
    }

    @Override
    public double getFeederKS() {
        return 0.0;
    }

    @Override
    public double getFeederKV() {
        return 0.3;
    }

    @Override
    public double getFeederKA() {
        return 0.000;
    }

    @Override
    public double getFeederKP() {
        return 0.1;
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
        return 3.0; // 5:1 gear ratio
    }

    // Indexer motor config
    @Override
    public int getIndexerCanId() {
        return 20;
    }

    @Override
    public boolean getIsIndexerInverted() {
        return true;
    }

    @Override
    public double getIndexerSupplyCurrentLimit() {
        return 90.0;
    }

    @Override
    public double getIndexerSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getIndexerSupplyCurrentLimitLowerLimit() {
        return 90.0;
    }

    @Override
    public double getIndexerStatorCurrentLimit() {
        return 90.0;
    }

    @Override
    public double getIndexerPeakForwardTorqueCurrent() {
        return 90.0;
    }

    @Override
    public double getIndexerPeakReverseTorqueCurrent() {
        return -90.0;
    }

    @Override
    public double getIndexerKS() {
        return 0.0;
    }

    @Override
    public double getIndexerKV() {
        return 0.3;
    }

    @Override
    public double getIndexerKA() {
        return 0.0;
    }

    @Override
    public double getIndexerKP() {
        return 0.1;
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
        return 3.0; // 5:1 gear ratio
    }

    @Override
    public double getHoodAngleToleranceRotations() {
        return 2.0 / 360.0;
    }

    @Override
    public double getTurretAngleToleranceRotations() {
        // Match hood tolerance by default
        return 2.0/360.0;
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
