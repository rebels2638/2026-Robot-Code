// package frc.robot.commands;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.shooter.Shooter;

// public class RunShooterFeeder extends Command {
//     private final Shooter shooter = Shooter.getInstance();
//     private final Supplier<Double> velocityRotationsPerSec;

//     // no requirements because it is expect that a requirement is added in the parent command 
//     // this allows for multiple parallel shooter calls to different shooter motors
//     public RunShooterFeeder(double velocityRotationsPerSec) {
//         this.velocityRotationsPerSec = () -> velocityRotationsPerSec;
//     }

//     public RunShooterFeeder(Supplier<Double> velocityRotationsPerSec) {
//         this.velocityRotationsPerSec = velocityRotationsPerSec;
//     } 

//     @Override
//     public void initialize() {
//         shooter.setFeedVelocity(velocityRotationsPerSec.get());
//     }

//     @Override
//     public void execute() {
//         shooter.setFeedVelocity(velocityRotationsPerSec.get());
//     }

//     @Override
//     public boolean isFinished() {
//         return shooter.isFeederAtSetpoint();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         if (interrupted) {
//             shooter.setFeedVelocity(0);
//         }
//     }
// }
