// package frc.robot.commands;

// import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.robot.subsystems.shooter.Shooter;

// public class DistanceShotWindup extends ParallelCommandGroup {
    
//     private static final Shooter shooter = Shooter.getInstance();

//     private static final LoggedNetworkNumber distanceDashboardNumber = new LoggedNetworkNumber("TunableShot/distanceMeters", 5);
//     private static final LoggedNetworkNumber feedVelocityDashboardNumber = new LoggedNetworkNumber("TunableShot/feedVelocityRotationsPerSec", 35);

//     public DistanceShotWindup(double distanceMeters) {
//         super(
//             new RunShooterFeeder(feedVelocityDashboardNumber.get()),
//             new RunShooterFlywheel(Shooter.getInstance().getLerpTable().get(distanceMeters).get(1, 0)),
//             new RunShooterHood(new Rotation2d((2*Math.PI)*Shooter.getInstance().getLerpTable().get(distanceMeters).get(0, 0)))
//         );
//         addRequirements(shooter);
//     }

//     public DistanceShotWindup() {
//         super(
//             new RunShooterFeeder(feedVelocityDashboardNumber.get()),
//             new RunShooterFlywheel(Shooter.getInstance().getLerpTable().get(distanceDashboardNumber.get()).get(1, 0)),
//             new RunShooterHood(new Rotation2d((2*Math.PI)*Shooter.getInstance().getLerpTable().get(distanceDashboardNumber.get()).get(0, 0)))
//         );
//         addRequirements(shooter);
//     }
// }
