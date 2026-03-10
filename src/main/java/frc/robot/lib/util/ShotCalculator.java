package frc.robot.lib.util;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import java.util.function.DoubleUnaryOperator;

public class ShotCalculator {

    private static final double GRAVITY = 9.81; // m/s^2

    public record ShotData(
        Rotation2d targetFieldYaw,
        Rotation2d hoodPitch,
        double flywheelRPS,
        double exitVelocity,
        double flightTime,
        double effectiveDistance,
        Translation2d compensatedTargetPosition
    ) {}

    public static ShotData calculate(
        Translation3d targetLocation,
        Translation3d shooterPosition, 
        ChassisSpeeds fieldRelativeSpeeds,
        InterpolatingMatrixTreeMap<Double, N2, N1> lerpTable,
        double latencyCompensationSeconds,
        DoubleUnaryOperator rpsToExitVelocity,
        Translation2d shooterOffsetFromRobotCenter,
        Rotation2d robotHeading
    ) {
        // Calculate shooter's tangential velocity due to robot rotation (omega)
        // In robot frame: v_tangential = omega cross r = (-omega * dy, omega * dx)
        double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
        double dx = shooterOffsetFromRobotCenter.getX();
        double dy = shooterOffsetFromRobotCenter.getY();
        
        // Tangential velocity in robot frame
        double tangentialVxRobot = -omega * dy;
        double tangentialVyRobot = omega * dx;
        
        // Rotate tangential velocity to field frame
        double cos = robotHeading.getCos();
        double sin = robotHeading.getSin();
        double tangentialVxField = tangentialVxRobot * cos - tangentialVyRobot * sin;
        double tangentialVyField = tangentialVxRobot * sin + tangentialVyRobot * cos;
        
        // Total shooter velocity in field frame (linear + tangential from rotation)
        // double shooterVxField = fieldRelativeSpeeds.vxMetersPerSecond + tangentialVxField;
        // double shooterVyField = fieldRelativeSpeeds.vyMetersPerSecond + tangentialVyField;
        
        double shooterVxField = fieldRelativeSpeeds.vxMetersPerSecond;
        double shooterVyField = fieldRelativeSpeeds.vyMetersPerSecond;
        

        // Iteratively solve for correct distance and flight time (robot reference frame approach)
        // Use HORIZONTAL distance (2D) as that's what the lerp table expects
        double shooterDistanceToTarget = shooterPosition.toTranslation2d().getDistance(targetLocation.toTranslation2d());
        
        double shotFlightTime = 0.0;
        double targetX = targetLocation.getX();
        double targetY = targetLocation.getY();
        double targetHeight = targetLocation.getZ();
        
        // Iterate to converge (robot frame: shot velocity is relative to robot, target appears to move)
        for (int i = 0; i < 30; i++) {
            
            // Get shooter settings for current distance estimate
            double hoodAngleRotations = lerpTable.get(shooterDistanceToTarget).get(0, 0);
            double flywheelRPS = lerpTable.get(shooterDistanceToTarget).get(1, 0);
            double exitVelocity = rpsToExitVelocity.applyAsDouble(flywheelRPS);
            double launchAngle = hoodAngleRotations * 2 * Math.PI; // Convert to radians
            
            // Calculate velocity components (relative to robot)
            double exitVelocityHorizontal = exitVelocity * Math.cos(launchAngle);
            double exitVelocityVertical = exitVelocity * Math.sin(launchAngle);
            
            // Calculate flight time using projectile motion physics
            // Solve: targetHeight = shooterHeight + vz0*t - 0.5*g*t^2
            // Rearranged: 0.5*g*t^2 - vz0*t + (shooterHeight - targetHeight) = 0
            // Using quadratic formula: t = (vz0 + sqrt(vz0^2 - 2*g*(shooterHeight - targetHeight))) / g
            double shooterHeight = shooterPosition.getZ();
            double deltaHeight = targetHeight - shooterHeight;
            
            // Flight time from vertical motion (projectile hits target height)
            double discriminant = exitVelocityVertical * exitVelocityVertical - 2 * GRAVITY * deltaHeight;
            if (discriminant < 0) {
                // Can't reach target (would require negative time), use simplified calculation
                shotFlightTime = shooterDistanceToTarget / exitVelocityHorizontal;
            } else {
                shotFlightTime = (exitVelocityVertical + Math.sqrt(discriminant)) / GRAVITY;
            }
            
            // In robot frame, target appears to move. Predict where it will appear to be
            // Use total shooter velocity (includes tangential component from omega)
            double vxDisplacement = shotFlightTime * shooterVxField + latencyCompensationSeconds * shooterVxField;
            double vyDisplacement = shotFlightTime * shooterVyField + latencyCompensationSeconds * shooterVyField;
            
            targetX = targetLocation.getX() - vxDisplacement;
            targetY = targetLocation.getY() - vyDisplacement;
            
            // Recalculate distance to compensated target
            double oldDistance = shooterDistanceToTarget;
            shooterDistanceToTarget = shooterPosition.toTranslation2d().getDistance(new Translation2d(targetX, targetY));
            
            double distanceChange = Math.abs(shooterDistanceToTarget - oldDistance);
            
            // Stop iterating if converged (distance change < 1cm)
            if (distanceChange < 0.01) {
                break;
            }
        }


        
        double shooterAngleToTarget = Math.atan2(targetY - shooterPosition.getY(), targetX - shooterPosition.getX());
        
        // Get final settings for this distance
        double hoodAngleRotations = lerpTable.get(shooterDistanceToTarget).get(0, 0);
        double flywheelVelocityRPS = lerpTable.get(shooterDistanceToTarget).get(1, 0);
        double exitVelocity = rpsToExitVelocity.applyAsDouble(flywheelVelocityRPS);
        
        return new ShotData(
            new Rotation2d(shooterAngleToTarget),
            new Rotation2d(hoodAngleRotations * 2 * Math.PI),
            flywheelVelocityRPS,
            exitVelocity,
            shotFlightTime,
            shooterDistanceToTarget,
            new Translation2d(targetX, targetY)
        );
    }
}
