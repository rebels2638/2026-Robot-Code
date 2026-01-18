package frc.robot.commands.shooter;
// package frc.robot.commands;

// import java.util.function.Supplier;

// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// import edu.wpi.first.math.InterpolatingMatrixTreeMap;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N2;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotState;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.swerve.SwerveDrive;
// import frc.robot.constants.Constants;

// public class MovingShotWindup extends Command {

//     private static final double GRAVITY = 9.81; // m/s^2
//     private static final LoggedNetworkNumber LATENCY_COMPENSATION_SECONDS = new LoggedNetworkNumber("MovingShotWindup/latencyCompensationSeconds",
//         switch (Constants.currentMode) {
//             case COMP -> 0.0;
//             case SIM -> 0.0;
//             case PROTO -> 0.0;
//             case REPLAY -> 0.0;
//             default -> 0.0;
//         }
//     );

//     private final Shooter shooter = Shooter.getInstance();
//     private final RobotState robotState = RobotState.getInstance();
//     private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

//     private final InterpolatingMatrixTreeMap<Double, N2, N1> lerpTable = shooter.getLerpTable();
//     private final PIDController rotationController = swerveDrive.getRotationController();

//     private final Translation2d targetTranslation;
//     private final double targetHeight;
//     private final Supplier<ChassisSpeeds> desiredSwerveSpeedsSupplier;

//     private boolean isShotValid = false;
//     private double maxRobotTranslationalVeloDuringShot;

//     public MovingShotWindup(Translation3d targetTranslation, Supplier<ChassisSpeeds> desiredFieldRelativeSwerveSpeedsSupplier, double maxRobotTranslationalVeloDuringShot) {
//         this.targetTranslation = targetTranslation.toTranslation2d();
//         this.targetHeight = targetTranslation.getZ();
//         this.desiredSwerveSpeedsSupplier = desiredFieldRelativeSwerveSpeedsSupplier;

//         this.maxRobotTranslationalVeloDuringShot = maxRobotTranslationalVeloDuringShot;

//         addRequirements(shooter, swerveDrive);
//     }

//     @Override
//     public void initialize() {
//         isShotValid = false;
//         Logger.recordOutput("MovingShotWindup/isActive", true);
//     }

//     @Override
//     public void execute() {

//         Pose2d currentPose = robotState.getEstimatedPose();
//         ChassisSpeeds currentSpeeds = robotState.getFieldRelativeSpeeds();

//         Logger.recordOutput("MovingShotWindup/currentPose", currentPose);
//         Logger.recordOutput("MovingShotWindup/currentSpeeds", currentSpeeds);

//         double speedsMagnitude = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

//         // Get shooter position without hood angle rotation
//         Pose3d robotPose3d = new Pose3d(robotState.getEstimatedPose());
//         Pose3d shooterPose = robotPose3d.plus(
//             new Transform3d(new Pose3d(), shooter.getShooterRelativePose())
//         );

//         // Iteratively solve for correct distance and flight time (robot reference frame approach)
//         // Use HORIZONTAL distance (2D) as that's what the lerp table expects
//         double shooterDistanceToTarget = shooterPose.getTranslation().toTranslation2d().getDistance(targetTranslation);

//         Logger.recordOutput("MovingShotWindup/initialShooterToTargetDistance2D", shooterDistanceToTarget);
//         Logger.recordOutput("MovingShotWindup/initialShooterToTargetDistance3D",
//             shooterPose.getTranslation().getDistance(new Translation3d(targetTranslation.getX(), targetTranslation.getY(), this.targetHeight)));

//         double shotFlightTime = 0.0;
//         double targetX = targetTranslation.getX();
//         double targetY = targetTranslation.getY();

//         // Iterate to converge (robot frame: shot velocity is relative to robot, target appears to move)
//         for (int i = 0; i < 30; i++) {
//             Logger.recordOutput("MovingShotWindup/iteration", i);
//             Logger.recordOutput("MovingShotWindup/iterationDistance", shooterDistanceToTarget);

//             // Get shooter settings for current distance estimate
//             double hoodAngleRotations = lerpTable.get(shooterDistanceToTarget).get(0, 0);
//             double flywheelRPS = lerpTable.get(shooterDistanceToTarget).get(1, 0);
//             double exitVelocity = shooter.calculateShotExitVelocityMetersPerSec(flywheelRPS);
//             double launchAngle = hoodAngleRotations * 2 * Math.PI; // Convert to radians

//             Logger.recordOutput("MovingShotWindup/iterationHoodAngleDeg", hoodAngleRotations * 360);
//             Logger.recordOutput("MovingShotWindup/iterationExitVelocity", exitVelocity);

//             // Calculate velocity components (relative to robot)
//             double exitVelocityHorizontal = exitVelocity * Math.cos(launchAngle);
//             double exitVelocityVertical = exitVelocity * Math.sin(launchAngle);

//             // Calculate flight time using projectile motion physics
//             // Solve: targetHeight = shooterHeight + vz0*t - 0.5*g*t^2
//             // Rearranged: 0.5*g*t^2 - vz0*t + (shooterHeight - targetHeight) = 0
//             // Using quadratic formula: t = (vz0 + sqrt(vz0^2 - 2*g*(shooterHeight - targetHeight))) / g
//             double shooterHeight = shooterPose.getZ();
//             double deltaHeight = this.targetHeight - shooterHeight;

//             // Flight time from vertical motion (projectile hits target height)
//             double discriminant = exitVelocityVertical * exitVelocityVertical - 2 * GRAVITY * deltaHeight;
//             if (discriminant < 0) {
//                 // Can't reach target (would require negative time), use simplified calculation
//                 shotFlightTime = shooterDistanceToTarget / exitVelocityHorizontal;
//                 Logger.recordOutput("MovingShotWindup/discriminantZero", true);
//             } else {
//                 shotFlightTime = (exitVelocityVertical + Math.sqrt(discriminant)) / GRAVITY;
//                 Logger.recordOutput("MovingShotWindup/discriminantZero", false);
//             }

//             Logger.recordOutput("MovingShotWindup/iterationFlightTime", shotFlightTime);


//             // In robot frame, target appears to move. Predict where it will appear to be
//             double vxDisplacement = shotFlightTime * currentSpeeds.vxMetersPerSecond + LATENCY_COMPENSATION_SECONDS.get() * currentSpeeds.vxMetersPerSecond;
//             double vyDisplacement = shotFlightTime * currentSpeeds.vyMetersPerSecond + LATENCY_COMPENSATION_SECONDS.get() * currentSpeeds.vyMetersPerSecond;

//             targetX = targetTranslation.getX() - vxDisplacement;
//             targetY = targetTranslation.getY() - vyDisplacement;

//             // Recalculate distance to compensated target
//             double oldDistance = shooterDistanceToTarget;
//             shooterDistanceToTarget = shooterPose.getTranslation().toTranslation2d().getDistance(new Translation2d(targetX, targetY));

//             double distanceChange = Math.abs(shooterDistanceToTarget - oldDistance);
//             Logger.recordOutput("MovingShotWindup/iterationDistanceChange", distanceChange);

//             // Stop iterating if converged (distance change < 1cm)
//             if (distanceChange < 0.01) {
//                 Logger.recordOutput("MovingShotWindup/iterationsUsed", i + 1);
//                 break;
//             }
//         }

//         double shooterAngleToTarget = Math.atan2(targetY - shooterPose.getTranslation().getY(), targetX - shooterPose.getTranslation().getX());

//         // Set shooter to optimal settings for this distance
//         double hoodAngleRotations = lerpTable.get(shooterDistanceToTarget).get(0, 0);
//         double flywheelVelocityRPS = lerpTable.get(shooterDistanceToTarget).get(1, 0);

//         shooter.setHoodAngle(new Rotation2d((Math.PI*2)*hoodAngleRotations));
//         shooter.setShotVelocity(flywheelVelocityRPS);
//         shooter.setFeedVelocity(35);

//         // Aim robot at predicted target
//         ChassisSpeeds speeds = desiredSwerveSpeedsSupplier.get();
//         speeds.omegaRadiansPerSecond = rotationController.calculate(shooterPose.getRotation().getZ(), shooterAngleToTarget);

//         swerveDrive.driveFieldRelative(speeds);

//         isShotValid = speedsMagnitude <= maxRobotTranslationalVeloDuringShot &&
//             shooterDistanceToTarget >= shooter.getMinShotDistFromShooterMeters() &&
//             shooterDistanceToTarget <= shooter.getMaxShotDistFromShooterMeters();

//         // Log diagnostic information
//         Logger.recordOutput("MovingShotWindup/shooterAngleToTarget", shooterAngleToTarget);
//         Logger.recordOutput("MovingShotWindup/shooterDistanceToTarget", shooterDistanceToTarget);
//         Logger.recordOutput("MovingShotWindup/inputTargetPosition", new Pose3d(new Translation3d(targetTranslation.getX(), targetTranslation.getY(), targetHeight), new Rotation3d()));
//         Logger.recordOutput("MovingShotWindup/offsetTargetPosition", new Pose2d(new Translation2d(targetX, targetY), new Rotation2d()));
//         Logger.recordOutput("MovingShotWindup/shotFlightTime", shotFlightTime);
//         Logger.recordOutput("MovingShotWindup/shooterHeight", shooterPose.getZ());
//         Logger.recordOutput("MovingShotWindup/shooterYaw", shooterPose.getRotation().getZ());
//         Logger.recordOutput("MovingShotWindup/targetHeight", this.targetHeight);
//         Logger.recordOutput("MovingShotWindup/deltaHeight", this.targetHeight - shooterPose.getZ());
//         Logger.recordOutput("MovingShotWindup/compensatedTargetX", targetX);
//         Logger.recordOutput("MovingShotWindup/compensatedTargetY", targetY);
//         Logger.recordOutput("MovingShotWindup/hoodAngleRotations", hoodAngleRotations);
//         Logger.recordOutput("MovingShotWindup/hoodAngleDegrees", hoodAngleRotations * 360);
//         Logger.recordOutput("MovingShotWindup/flywheelVelocityRPS", flywheelVelocityRPS);
//         Logger.recordOutput("MovingShotWindup/exitVelocity", shooter.calculateShotExitVelocityMetersPerSec(flywheelVelocityRPS));
//         Logger.recordOutput("MovingShotWindup/isShotValid", isShotValid);
//         Logger.recordOutput("MovingShotWindup/isFinished", isFinished());
//     }


//     @Override
//     public boolean isFinished() {
//         boolean finished =
//             isShotValid &&
//             rotationController.atSetpoint() &&
//             shooter.isHoodAtSetpoint() &&
//             shooter.isFlywheelAtSetpoint() &&
//             shooter.isFeederAtSetpoint();

//         Logger.recordOutput("MovingShotWindup/isRotationControllerAtSetpoint", rotationController.atSetpoint());

//         return finished;
//     }

//     @Override
//     public void end(boolean interrupted) {


//         isShotValid = false;
//         Logger.recordOutput("MovingShotWindup/endTime", Timer.getFPGATimestamp());

//         Logger.recordOutput("MovingShotWindup/isActive", false);
//     }
// }
