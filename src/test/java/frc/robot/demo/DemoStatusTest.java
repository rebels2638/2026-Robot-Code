package frc.robot.demo;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import org.junit.jupiter.api.Test;

class DemoStatusTest {
    @Test
    void tagStatus_reportsTheFirstThingStoppingTheShot() {
        assertTrue(DemoController.describeTagStatus(false, true, 3.0, Optional.empty(), true, true, true, true, false, false)
            .startsWith("NO TARGET"));
        assertTrue(DemoController.describeTagStatus(true, false, 0.4, Optional.empty(), true, true, true, true, false, false)
            .startsWith("OUT OF RANGE"));
        assertEquals("CEILING", DemoController.describeTagStatus(
            true, true, 3.0, Optional.of("CEILING"), true, true, true, true, false, false));
        assertEquals("SHOOTER NOT READY: turret, flywheel", DemoController.describeTagStatus(
            true, true, 3.0, Optional.empty(), false, true, false, false, false, false));
        assertEquals("READY: hold RT", DemoController.describeTagStatus(
            true, true, 3.0, Optional.empty(), true, true, true, true, false, false));
        assertEquals("CALIBRATING: FIRING", DemoController.describeTagStatus(
            true, true, 3.0, Optional.empty(), true, true, true, true, true, true));
    }

    @Test
    void settingsClamp_rejectsNonFiniteAndOutOfRange() {
        assertEquals(0.25, DemoSettings.clampOr(Double.NaN, 0.05, 0.5, 0.25), 1e-12);
        assertEquals(0.5, DemoSettings.clampOr(1.0, 0.05, 0.5, 0.25), 1e-12);
        assertEquals(0.05, DemoSettings.clampOr(-3.0, 0.05, 0.5, 0.25), 1e-12);
        assertEquals(0.3, DemoSettings.clampOr(0.3, 0.05, 0.5, 0.25), 1e-12);
    }

    @Test
    void modeToggles() {
        assertEquals(DemoMode.TAG, DemoMode.MANUAL.toggled());
        assertEquals(DemoMode.MANUAL, DemoMode.TAG.toggled());
    }
}
