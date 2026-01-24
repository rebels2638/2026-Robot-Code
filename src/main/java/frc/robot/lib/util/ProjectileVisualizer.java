package frc.robot.lib.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Visualizes a projectile's trajectory using physics simulation.
 * Simulates 2D projectile motion with gravity and logs the position to AdvantageScope.
 * The command ends when the projectile hits the ground (z < 0).
 */
public class ProjectileVisualizer extends Command {
    
    // Physics constants
    private static final double GRAVITY = 9.81; // m/s^2
    
    // Initial conditions
    private final double initialVx; // Robot velocity in x direction (m/s)
    private final double initialVy; // Robot velocity in y direction (m/s)
    private final double launchVelocity; // Launch velocity magnitude (m/s)
    private final Pose3d initialPose; // Initial shooter pose (position and orientation)
    
    // State variables
    private Translation3d currentPosition;
    private double startTime;
    private double launchAngle; // Angle above horizontal (radians)
    private double launchYaw; // Yaw direction (radians)
    private double vx0; // Total initial velocity in x
    private double vy0; // Total initial velocity in y
    private double vz0; // Initial velocity in z
    
    /**
     * Creates a new ProjectileVisualizer command.
     * 
     * @param robotVx Robot velocity in x direction (field-relative, m/s)
     * @param robotVy Robot velocity in y direction (field-relative, m/s)
     * @param launchVelocity Magnitude of launch velocity (m/s)
     * @param launchPose Initial pose of the shooter (includes position and orientation)
     */

    // TODO: add feature to vary z terminal height in the constructor
    public ProjectileVisualizer(double robotVx, double robotVy, double launchVelocity, Pose3d launchPose) {
        this.initialVx = robotVx;
        this.initialVy = robotVy;
        this.launchVelocity = launchVelocity;
        this.initialPose = new Pose3d(
            launchPose.getX(),
            launchPose.getY(),
            launchPose.getZ(),
            new Rotation3d(
                launchPose.getRotation().getX(),
                launchPose.getRotation().getY(),
                launchPose.getRotation().getZ()
            )
        );
    }
    
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        
        // Extract launch angle from the pose's pitch (rotation around Y-axis)
        Rotation3d rotation = initialPose.getRotation();
        launchAngle = rotation.getY(); // Pitch angle (elevation)
        launchYaw = rotation.getZ(); // Yaw angle (direction)
        
        // Calculate initial velocity components from launch velocity
        // The launch velocity is decomposed based on the shooter's orientation
        double launchVxComponent = launchVelocity * Math.cos(launchAngle) * Math.cos(launchYaw);
        double launchVyComponent = launchVelocity * Math.cos(launchAngle) * Math.sin(launchYaw);
        double launchVzComponent = launchVelocity * Math.sin(launchAngle);
        
        // Combine robot velocity with launch velocity
        vx0 = initialVx + launchVxComponent;
        vy0 = initialVy + launchVyComponent;
        vz0 = launchVzComponent;
        
        // Set initial position
        Translation3d initialTranslation = initialPose.getTranslation();
        currentPosition = initialTranslation;
        
        // Log initial state
        Logger.recordOutput("ProjectileVisualizer/InitialPose", initialPose);
        Logger.recordOutput("ProjectileVisualizer/LaunchAngleDegrees", Math.toDegrees(launchAngle));
        Logger.recordOutput("ProjectileVisualizer/LaunchVelocity", launchVelocity);
    }
    
    @Override
    public void execute() {
        // Calculate elapsed time
        double t = Timer.getFPGATimestamp() - startTime;
        
        // Calculate current position using kinematic equations
        // x(t) = x0 + vx0 * t
        // y(t) = y0 + vy0 * t
        // z(t) = z0 + vz0 * t - 0.5 * g * t^2
        double x = initialPose.getX() + vx0 * t;
        double y = initialPose.getY() + vy0 * t;
        double z = MathUtil.clamp(initialPose.getZ() + vz0 * t - 0.5 * GRAVITY * t * t, 0, Double.POSITIVE_INFINITY);
        
        // Update current position
        currentPosition = new Translation3d(x, y, z);
        
        // Create a pose for visualization (keeping the same rotation as initial)
        Pose3d currentPose = new Pose3d(currentPosition, initialPose.getRotation());
        
        // Log to AdvantageScope
        Logger.recordOutput("ProjectileVisualizer/CurrentPose", currentPose);
        Logger.recordOutput("ProjectileVisualizer/CurrentPosition", currentPosition);
        Logger.recordOutput("ProjectileVisualizer/ElapsedTime", t);
        Logger.recordOutput("ProjectileVisualizer/Height", z);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Log final position
        Logger.recordOutput("ProjectileVisualizer/FinalPosition", currentPosition);
        Logger.recordOutput("ProjectileVisualizer/Interrupted", interrupted);
        
        // Clear the visualization after a brief moment (or immediately)
        // Logger.recordOutput("ProjectileVisualizer/CurrentPose", new Pose3d());
    }
    
    @Override
    public boolean isFinished() {
        // End when projectile reaches negative z (hits the ground)
        return currentPosition.getZ() <= 0.46;
    }
}