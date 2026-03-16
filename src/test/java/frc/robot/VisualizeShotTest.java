package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import frc.robot.constants.Constants;
import frc.robot.lib.util.ballistics.ProjectileVisualizer;
import java.lang.reflect.Field;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VisualizeShotTest {
    @BeforeEach
    void setUp() {
        ProjectileVisualizer.clearProjectiles();
    }

    @Test
    // VisualizeShot only enqueues projectiles in SIM mode.
    void explicitVelocityConstructor_respectsSimulationGate() {
        int before = getActiveProjectileCountReflective();

        assertDoesNotThrow(() -> new VisualizeShot(9.0));

        int after = getActiveProjectileCountReflective();
        assertEquals(before + getExpectedProjectileDelta(), after);
    }

    @Test
    // Default constructor should follow the same SIM-only contract.
    void defaultConstructor_respectsSimulationGate() {
        int before = getActiveProjectileCountReflective();

        assertDoesNotThrow(() -> new VisualizeShot());

        int after = getActiveProjectileCountReflective();
        assertEquals(before + getExpectedProjectileDelta(), after);
    }

    private int getExpectedProjectileDelta() {
        return Constants.currentMode == Constants.Mode.SIM ? 1 : 0;
    }

    @SuppressWarnings("unchecked")
    private int getActiveProjectileCountReflective() {
        try {
            Field projectilesField = ProjectileVisualizer.class.getDeclaredField("projectiles");
            projectilesField.setAccessible(true);
            List<Object> projectiles = (List<Object>) projectilesField.get(ProjectileVisualizer.getInstance());
            return projectiles.size();
        } catch (ReflectiveOperationException e) {
            throw new AssertionError("Failed to inspect projectile count for test", e);
        }
    }
}
