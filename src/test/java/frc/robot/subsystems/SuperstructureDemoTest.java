package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import frc.robot.subsystems.Superstructure.ManualFeedReason;
import org.junit.jupiter.api.Test;

class SuperstructureDemoTest {
    private static ManualFeedReason decide(
        boolean force, boolean hood, boolean turret, boolean flywheel, double elapsed, double measured, double setpoint
    ) {
        return Superstructure.decideManualFeed(force, hood, turret, flywheel, elapsed, measured, setpoint, 1.5, 0.8);
    }

    @Test
    void manualFeed_readyWhenEverythingAtSetpoint() {
        assertEquals(ManualFeedReason.READY, decide(false, true, true, true, 0.2, 22.0, 22.0));
    }

    @Test
    void manualFeed_waitsWhileSpinningUp() {
        assertEquals(ManualFeedReason.NOT_YET, decide(false, true, false, true, 0.5, 22.0, 22.0));
        assertEquals(ManualFeedReason.NOT_YET, decide(false, false, false, false, 2.0, 10.0, 22.0));
    }

    @Test
    void manualFeed_timeoutFallbackNeedsEightyPercentSpeed() {
        assertEquals(ManualFeedReason.TIMEOUT, decide(false, false, false, false, 1.6, 18.0, 22.0));
        assertEquals(ManualFeedReason.NOT_YET, decide(false, true, true, false, 1.6, 0.0, 0.0));
    }

    @Test
    void manualFeed_forceFeedAlwaysFeeds() {
        assertEquals(ManualFeedReason.FORCED, decide(true, false, false, false, 0.0, 0.0, 22.0));
    }

    @Test
    void demoTagIsNotAPassTarget() {
        assertFalse(Superstructure.TargetState.DEMO_TAG.isPassTarget());
        assertFalse(Superstructure.TargetState.DEMO_TAG.isAlliancePassTarget());
    }
}
