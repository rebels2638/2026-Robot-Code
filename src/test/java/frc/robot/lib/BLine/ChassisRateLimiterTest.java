package frc.robot.lib.BLine;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class ChassisRateLimiterTest {
    @Test
    void limit_clampsTranslationalAccelerationMagnitude() {
        ChassisSpeeds limited = ChassisRateLimiter.limit(
            new ChassisSpeeds(3.0, 4.0, 0.0),
            new ChassisSpeeds(),
            1.0,
            2.0,
            100.0,
            10.0,
            10.0
        );

        assertEquals(1.2, limited.vxMetersPerSecond, 1e-9);
        assertEquals(1.6, limited.vyMetersPerSecond, 1e-9);
        assertEquals(0.0, limited.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void limit_clampsAngularAcceleration() {
        ChassisSpeeds limited = ChassisRateLimiter.limit(
            new ChassisSpeeds(0.0, 0.0, 10.0),
            new ChassisSpeeds(),
            0.5,
            100.0,
            4.0,
            10.0,
            20.0
        );

        assertEquals(0.0, limited.vxMetersPerSecond, 1e-9);
        assertEquals(0.0, limited.vyMetersPerSecond, 1e-9);
        assertEquals(2.0, limited.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void limit_appliesVelocityClampBeforeAccelerationClamp() {
        ChassisSpeeds limited = ChassisRateLimiter.limit(
            new ChassisSpeeds(10.0, 0.0, 0.0),
            new ChassisSpeeds(),
            1.0,
            10.0,
            100.0,
            4.0,
            10.0
        );

        assertEquals(4.0, limited.vxMetersPerSecond, 1e-9);
        assertEquals(0.0, limited.vyMetersPerSecond, 1e-9);
    }

    @Test
    void limit_whenDtIsNonPositiveOnlyAppliesVelocityClamp() {
        ChassisSpeeds limited = ChassisRateLimiter.limit(
            new ChassisSpeeds(6.0, 8.0, 10.0),
            new ChassisSpeeds(1.0, 1.0, 1.0),
            0.0,
            0.1,
            0.1,
            5.0,
            2.0
        );

        assertEquals(3.0, limited.vxMetersPerSecond, 1e-9);
        assertEquals(4.0, limited.vyMetersPerSecond, 1e-9);
        assertEquals(2.0, limited.omegaRadiansPerSecond, 1e-9);
    }
}
