package frc.robot.lib.util.ballistics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class BallisticsPhysicsTest {
    private static final double EPS = 1e-9;

    @Test
    // Stationary 2D projectile should only experience gravity.
    void computeDerivatives2D_zeroVelocity_returnsGravityOnly() {
        BallisticsPhysics.State2D state = new BallisticsPhysics.State2D(1.0, 2.0, 0.0, 0.0);

        BallisticsPhysics.Derivatives2D derivatives = BallisticsPhysics.computeDerivatives2D(state, 100.0);

        assertEquals(0.0, derivatives.dx(), EPS);
        assertEquals(0.0, derivatives.dz(), EPS);
        assertEquals(0.0, derivatives.dvx(), EPS);
        assertEquals(-BallisticsConstants.GRAVITY, derivatives.dvz(), EPS);
    }

    @Test
    // Stationary 3D projectile should only experience gravity.
    void computeDerivatives3D_zeroVelocity_returnsGravityOnly() {
        ProjectileState state = new ProjectileState(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);

        ProjectileState.Derivatives derivatives = BallisticsPhysics.computeDerivatives3D(state, 100.0);

        assertEquals(0.0, derivatives.dx(), EPS);
        assertEquals(0.0, derivatives.dy(), EPS);
        assertEquals(0.0, derivatives.dz(), EPS);
        assertEquals(0.0, derivatives.dvx(), EPS);
        assertEquals(0.0, derivatives.dvy(), EPS);
        assertEquals(-BallisticsConstants.GRAVITY, derivatives.dvz(), EPS);
    }

    @Test
    // Simple model should keep horizontal velocity constant and only accelerate vertically under gravity.
    void computeDerivatives2D_simpleModel_returnsPhysicsOneTerms() {
        BallisticsPhysics.State2D state = new BallisticsPhysics.State2D(0.0, 1.0, 7.0, 5.0);

        BallisticsPhysics.Derivatives2D derivatives =
            BallisticsPhysics.computeDerivatives2D(state, 999.0, BallisticsModel.SIMPLE);

        assertEquals(7.0, derivatives.dx(), EPS);
        assertEquals(5.0, derivatives.dz(), EPS);
        assertEquals(0.0, derivatives.dvx(), EPS);
        assertEquals(-BallisticsConstants.GRAVITY, derivatives.dvz(), EPS);
    }

    @Test
    // Backspin should increase vertical accel relative to no-spin, topspin should decrease it.
    void computeDerivatives2D_backspinDirection_changesVerticalAccelerationDirection() {
        BallisticsPhysics.State2D state = new BallisticsPhysics.State2D(0.0, 2.0, 15.0, 0.0);

        BallisticsPhysics.Derivatives2D noSpin = BallisticsPhysics.computeDerivatives2D(state, 0.0);
        BallisticsPhysics.Derivatives2D backspin = BallisticsPhysics.computeDerivatives2D(state, 200.0);
        BallisticsPhysics.Derivatives2D topspin = BallisticsPhysics.computeDerivatives2D(state, -200.0);

        assertTrue(backspin.dvz() > noSpin.dvz());
        assertTrue(topspin.dvz() < noSpin.dvz());
    }

    @Test
    // One integration step should move position forward and reduce vertical velocity under gravity.
    void integrateStep2D_advancesState() {
        BallisticsPhysics.State2D state = new BallisticsPhysics.State2D(0.0, 1.0, 12.0, 4.0);

        BallisticsPhysics.State2D next = BallisticsPhysics.integrateStep2D(state, 0.0, 0.02);

        assertTrue(next.x() > state.x());
        assertTrue(next.z() > state.z());
        assertTrue(next.vz() < state.vz());
    }

    @Test
    // Simple model integration should match the closed-form constant-vx trajectory exactly.
    void integrateStep2D_simpleModel_matchesClosedForm() {
        BallisticsPhysics.State2D state = new BallisticsPhysics.State2D(0.0, 1.0, 12.0, 4.0);
        double dt = 0.25;

        BallisticsPhysics.State2D next = BallisticsPhysics.integrateStep2D(state, 0.0, dt, BallisticsModel.SIMPLE);

        assertEquals(3.0, next.x(), EPS);
        assertEquals(1.0 + 4.0 * dt - 0.5 * BallisticsConstants.GRAVITY * dt * dt, next.z(), EPS);
        assertEquals(12.0, next.vx(), EPS);
        assertEquals(4.0 - BallisticsConstants.GRAVITY * dt, next.vz(), EPS);
    }

    @Test
    // Nominal trajectory should reach requested horizontal distance.
    void simulateToDistance_nominalShot_reachesTarget() {
        TrajectoryResult result = BallisticsPhysics.simulateToDistance(
            2.0,
            Math.toRadians(45.0),
            12.0,
            1.0,
            1.5,
            50.0,
            0.002
        );

        assertTrue(result.reachedTarget());
        assertEquals(2.0, result.finalDistance(), 1e-6);
        assertTrue(result.finalHeight() >= 0.0);
    }

    @Test
    // Very weak shot toward far target should report unreached.
    void simulateToDistance_unreachableShot_reportsNotReached() {
        TrajectoryResult result = BallisticsPhysics.simulateToDistance(
            50.0,
            Math.toRadians(20.0),
            3.0,
            1.0,
            2.0,
            0.0,
            0.002
        );

        assertFalse(result.reachedTarget());
        assertTrue(result.finalDistance() < 50.0);
    }

    @Test
    // Simple model should use the exact textbook parabola at the requested horizontal distance.
    void simulateToDistance_simpleModel_matchesClosedFormHeight() {
        double distance = 3.0;
        double angle = Math.toRadians(40.0);
        double exitVelocity = 10.0;
        double expectedTime = distance / (exitVelocity * Math.cos(angle));
        double expectedHeight =
            1.0 + exitVelocity * Math.sin(angle) * expectedTime - 0.5 * BallisticsConstants.GRAVITY * expectedTime * expectedTime;

        TrajectoryResult result = BallisticsPhysics.simulateToDistance(
            distance,
            angle,
            exitVelocity,
            1.0,
            0.0,
            999.0,
            0.002,
            BallisticsModel.SIMPLE
        );

        assertTrue(result.reachedTarget());
        assertEquals(expectedTime, result.flightTime(), EPS);
        assertEquals(expectedHeight, result.finalHeight(), EPS);
        assertEquals(distance, result.finalDistance(), EPS);
    }

    @Test
    // Unreached target should trigger the horizontal-velocity fallback flight-time estimate.
    void estimateFlightTime_unreachableShot_usesFallbackHorizontalEstimate() {
        double distance = 100.0;
        double angleRad = Math.toRadians(30.0);
        double exitVelocity = 1.0;
        double expected = distance / Math.max(1e-6, exitVelocity * Math.cos(angleRad));

        double actual = BallisticsPhysics.estimateFlightTime(
            distance,
            angleRad,
            exitVelocity,
            1.0,
            2.0,
            0.0
        );

        assertEquals(expected, actual, EPS);
    }

    @Test
    // Crossing target height while descending should return interpolated point on that plane.
    void simulateToHeight3D_crossingTargetHeight_returnsInterpolatedPoint() {
        Translation3d result = BallisticsPhysics.simulateToHeight3D(
            new Translation3d(0.0, 0.0, 1.0),
            8.0,
            0.0,
            6.0,
            0.0,
            1.0,
            0.002
        );

        assertEquals(1.0, result.getZ(), 1e-9);
        assertTrue(result.getX() > 0.0);
    }

    @Test
    // If requested height is never crossed, return terminal simulation state.
    void simulateToHeight3D_whenNoCrossing_returnsTerminalState() {
        Translation3d result = BallisticsPhysics.simulateToHeight3D(
            new Translation3d(0.0, 0.0, 1.0),
            4.0,
            0.0,
            3.0,
            0.0,
            10.0,
            0.002
        );

        assertTrue(result.getZ() < 10.0);
    }

    @Test
    // Simple 3D crossing should preserve constant horizontal velocity and hit the requested plane exactly.
    void simulateToHeight3D_simpleModel_matchesClosedFormCrossing() {
        Translation3d result = BallisticsPhysics.simulateToHeight3D(
            new Translation3d(0.5, -0.25, 1.0),
            6.0,
            -2.0,
            5.0,
            1000.0,
            1.0,
            0.002,
            BallisticsModel.SIMPLE
        );

        assertEquals(1.0, result.getZ(), EPS);
        assertTrue(result.getX() > 0.5);
        assertTrue(result.getY() < -0.25);
    }
}
