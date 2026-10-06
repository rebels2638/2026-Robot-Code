package frc.robot.demo;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.lib.util.ShotCalculator.ShotData;
import frc.robot.lib.util.ShotKinematicsUtil.ShooterKinematics;
import java.util.Optional;

/** What the Superstructure needs from the demo for manual fire and tag-mode shots. */
public interface DemoShotSource {
    /** Aim point (tag center plus offsets) in the odometry frame, or empty when there is no target. */
    Optional<Translation3d> getTargetLocation();

    /** Lerp table (distance -> hood deg, flywheel RPS, flight time) for a target at this height. */
    InterpolatingMatrixTreeMap<Double, N3, N1> getLerpTable(double targetHeightMeters);

    double getMinShotDistanceMeters();

    double getMaxShotDistanceMeters();

    /** Ball exit speed for a flywheel speed, from the demo's fitted model. */
    double exitSpeedForRps(double flywheelRps);

    /** Reason this tag shot must not fire (unreachable, flywheel limit, ceiling), or empty when it is allowed. */
    Optional<String> evaluateShotBlock(ShotData shotData, ShooterKinematics shooterKinematics, Translation3d targetLocation);

    double getManualHoodDegrees();

    /** Robot-relative turret angle for manual fire. */
    double getManualTurretDegrees();

    double getManualFlywheelRps();

    boolean isManualForceFeed();
}
