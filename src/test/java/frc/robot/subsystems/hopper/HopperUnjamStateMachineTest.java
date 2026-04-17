package frc.robot.subsystems.hopper;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.hopper.Hopper.UnjamState;
import frc.robot.subsystems.hopper.Hopper.UnjamTransition;

class HopperUnjamStateMachineTest {
    private static final int CYCLE_COUNT = 2;
    private static final double SEGMENT_DURATION = 0.1;
    private static final double COOLDOWN = 0.5;

    private static UnjamTransition resolve(
        UnjamState state,
        int segmentIndex,
        boolean debouncedStall,
        double segmentElapsed,
        double cooldownElapsed,
        boolean gateActive
    ) {
        return Hopper.resolveNextUnjamState(
            state,
            segmentIndex,
            debouncedStall,
            segmentElapsed,
            cooldownElapsed,
            CYCLE_COUNT,
            SEGMENT_DURATION,
            COOLDOWN,
            gateActive
        );
    }

    @Test
    void isStallCandidate_requiresBothHighCurrentAndLowVelocity() {
        assertTrue(Hopper.isStallCandidate(80.0, 0.5, 60.0, 1.0));
        assertFalse(Hopper.isStallCandidate(40.0, 0.5, 60.0, 1.0), "low current should not count");
        assertFalse(Hopper.isStallCandidate(80.0, 5.0, 60.0, 1.0), "non-stalled velocity should not count");
    }

    @Test
    void isStallCandidate_handlesNegativeSigns() {
        assertTrue(Hopper.isStallCandidate(-80.0, -0.2, 60.0, 1.0));
    }

    @Test
    void gateInactive_forcesIdle() {
        UnjamTransition result = resolve(UnjamState.UNJAMMING, 3, false, 0.0, 0.0, false);
        assertEquals(UnjamState.IDLE, result.nextState);
        assertEquals(0, result.nextSegmentIndex);
    }

    @Test
    void idle_noStall_staysIdle() {
        UnjamTransition result = resolve(UnjamState.IDLE, 0, false, 0.0, 0.0, true);
        assertEquals(UnjamState.IDLE, result.nextState);
        assertFalse(result.resetSegmentTimer);
    }

    @Test
    void idle_debouncedStall_entersUnjammingAtSegmentZero() {
        UnjamTransition result = resolve(UnjamState.IDLE, 0, true, 0.0, 0.0, true);
        assertEquals(UnjamState.UNJAMMING, result.nextState);
        assertEquals(0, result.nextSegmentIndex);
        assertTrue(result.resetSegmentTimer, "must stamp segment-start time when entering UNJAMMING");
    }

    @Test
    void unjamming_beforeSegmentElapses_holdsSegment() {
        UnjamTransition result = resolve(UnjamState.UNJAMMING, 1, false, SEGMENT_DURATION / 2.0, 0.0, true);
        assertEquals(UnjamState.UNJAMMING, result.nextState);
        assertEquals(1, result.nextSegmentIndex);
        assertFalse(result.resetSegmentTimer);
    }

    @Test
    void unjamming_afterSegmentElapses_advancesIndex() {
        UnjamTransition result = resolve(UnjamState.UNJAMMING, 0, false, SEGMENT_DURATION, 0.0, true);
        assertEquals(UnjamState.UNJAMMING, result.nextState);
        assertEquals(1, result.nextSegmentIndex);
        assertTrue(result.resetSegmentTimer);
    }

    @Test
    void unjamming_afterFinalSegment_entersCooldown() {
        // CYCLE_COUNT=2 means 2*N=4 segments (indices 0..3). Finishing index 3 triggers cooldown.
        UnjamTransition result = resolve(UnjamState.UNJAMMING, 3, false, SEGMENT_DURATION, 0.0, true);
        assertEquals(UnjamState.COOLDOWN, result.nextState);
        assertTrue(result.resetCooldownTimer);
    }

    @Test
    void cooldown_beforeExpiry_staysInCooldown() {
        UnjamTransition result = resolve(UnjamState.COOLDOWN, 0, true, 0.0, COOLDOWN / 2.0, true);
        assertEquals(UnjamState.COOLDOWN, result.nextState);
    }

    @Test
    void cooldown_afterExpiry_returnsToIdle() {
        UnjamTransition result = resolve(UnjamState.COOLDOWN, 0, false, 0.0, COOLDOWN, true);
        assertEquals(UnjamState.IDLE, result.nextState);
        assertEquals(0, result.nextSegmentIndex);
    }

    @Test
    void cooldown_ignoresDebouncedStall() {
        // Even if stall fires again during cooldown, we stay in cooldown; re-arm happens only
        // when we return to IDLE.
        UnjamTransition result = resolve(UnjamState.COOLDOWN, 0, true, 0.0, COOLDOWN / 4.0, true);
        assertEquals(UnjamState.COOLDOWN, result.nextState);
    }

    @Test
    void fullSequence_walksThroughAllSegmentsThenCooldownThenIdle() {
        UnjamState state = UnjamState.IDLE;
        int idx = 0;

        UnjamTransition t = resolve(state, idx, true, 0.0, 0.0, true);
        state = t.nextState;
        idx = t.nextSegmentIndex;
        assertEquals(UnjamState.UNJAMMING, state);
        assertEquals(0, idx);

        for (int expected = 1; expected < 2 * CYCLE_COUNT; expected++) {
            t = resolve(state, idx, false, SEGMENT_DURATION, 0.0, true);
            state = t.nextState;
            idx = t.nextSegmentIndex;
            assertEquals(UnjamState.UNJAMMING, state, "still unjamming at step " + expected);
            assertEquals(expected, idx);
        }

        t = resolve(state, idx, false, SEGMENT_DURATION, 0.0, true);
        state = t.nextState;
        idx = t.nextSegmentIndex;
        assertEquals(UnjamState.COOLDOWN, state);

        t = resolve(state, idx, false, 0.0, COOLDOWN, true);
        assertEquals(UnjamState.IDLE, t.nextState);
    }
}
