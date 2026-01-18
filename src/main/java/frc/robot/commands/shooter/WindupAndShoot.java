package frc.robot.commands.shooter;
// package frc.robot.commands;

// import java.util.function.Supplier;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.VisualizeShot;
// import frc.robot.constants.Constants;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.swerve.SwerveDrive;

// public class WindupAndShoot extends SequentialCommandGroup {

//     private final Shooter shooter = Shooter.getInstance();
//     private final SwerveDrive swerve = SwerveDrive.getInstance();

//     public WindupAndShoot(Supplier<ChassisSpeeds> desiredFieldRelativeSwerveSpeedsSupplier) {
//         super(
//             new MovingShotWindup(
//                 Constants.FieldConstants.kSHOOTER_TARGET,
//                 desiredFieldRelativeSwerveSpeedsSupplier,
//                 4.0
//             ).asProxy(),
//             new ParallelDeadlineGroup(
//                 new WaitCommand(0.12),
//                 new MovingShotWindup(
//                     Constants.FieldConstants.kSHOOTER_TARGET,
//                     desiredFieldRelativeSwerveSpeedsSupplier,
//                     4.0
//                 ).asProxy(),
//                 new InstantCommand(() -> new VisualizeShot()),
//                 new RunShooterIndexer(35).asProxy()
//             ),
//             new RunShooterIndexer(0),
//             new WaitCommand(1),
//             new RunShooterFlywheel(0),
//             new RunShooterFeeder(0)

//         );
//     }


// }
