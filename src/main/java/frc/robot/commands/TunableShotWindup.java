// package frc.robot.commands;

// import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.robot.subsystems.shooter.Shooter;

// public class TunableShotWindup extends ParallelCommandGroup {
    
//     private static final Shooter shooter = Shooter.getInstance();

//     private static final LoggedNetworkNumber angleDashboardNumber = new LoggedNetworkNumber("TunableShot/angleDegrees", 55);
//     private static final LoggedNetworkNumber velocityDashboardNumber = new LoggedNetworkNumber("TunableShot/flywheelVelocityRotationsPerSec", 60);
//     private static final LoggedNetworkNumber feedVelocityDashboardNumber = new LoggedNetworkNumber("TunableShot/feedVelocityRotationsPerSec", 35);

//     public TunableShotWindup() {
//         super(
//             new RunShooterHood(() -> Rotation2d.fromDegrees(angleDashboardNumber.get())),
//             new RunShooterFlywheel(() -> velocityDashboardNumber.get()),
//             new RunShooterFeeder(() -> feedVelocityDashboardNumber.get())
//         );
//         addRequirements(shooter);
//     }
// }
