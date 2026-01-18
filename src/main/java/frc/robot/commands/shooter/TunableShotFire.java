package frc.robot.commands.shooter;
// package frc.robot.commands;

// import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.VisualizeShot;
// import frc.robot.subsystems.shooter.Shooter;

// public class TunableShotFire extends SequentialCommandGroup {

//     private static final Shooter shooter = Shooter.getInstance();
//     private static final LoggedNetworkNumber indexerVelocityDashboardNumber = new LoggedNetworkNumber("TunableShot/indexerVelocityRotationsPerSec", 35);
//     private static final LoggedNetworkNumber indexerRunTimeDashboardNumber = new LoggedNetworkNumber("TunableShot/indexerRunTimeSecDashboardNumber", 0.12);
//     private static final LoggedNetworkNumber stopDelayTimeDashboardNumber = new LoggedNetworkNumber("TunableShot/stopDelayTimeSecDashboardNumber", 0.5);


//     public TunableShotFire() {
//         super(
//             // new InstantCommand(() -> System.out.println("TunableShotFire")),
//             new ParallelDeadlineGroup(
//                 new WaitCommand(indexerRunTimeDashboardNumber.get()),
//                 new RunShooterIndexer(() -> indexerVelocityDashboardNumber.get())
//             ),
//             new InstantCommand(() -> new VisualizeShot()),
//             new RunShooterIndexer(0),
//             new WaitCommand(stopDelayTimeDashboardNumber.get()),
//             new RunShooterFlywheel(0),
//             new RunShooterFeeder(0)
//         );
//         addRequirements(shooter);
//     }


// }
