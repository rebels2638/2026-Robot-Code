package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.constants.Constants;
import org.junit.jupiter.api.Test;

class VisionPerformanceGuardTest {
    @Test
    void keepsAllSamplesWhenQueueIsWithinBudget() {
        assertEquals(0, VisionIO.getLatestSampleStartIndex(0));
        assertEquals(0, VisionIO.getLatestSampleStartIndex(VisionIO.MAX_OBSERVATION_SAMPLES_PER_UPDATE));
    }

    @Test
    void dropsOldestSamplesWhenQueueBacklogExceedsBudget() {
        assertEquals(3, VisionIO.getLatestSampleStartIndex(VisionIO.MAX_OBSERVATION_SAMPLES_PER_UPDATE + 3));
        assertEquals(15, VisionIO.getLatestSampleStartIndex(20));
    }

    @Test
    void detailedPoseLoggingOnlyRunsInReplayOrVerboseModes() {
        assertFalse(Vision.isDetailedPoseLoggingEnabled(Constants.Mode.COMP, false));
        assertTrue(Vision.isDetailedPoseLoggingEnabled(Constants.Mode.REPLAY, false));
        assertTrue(Vision.isDetailedPoseLoggingEnabled(Constants.Mode.COMP, true));
    }
}
