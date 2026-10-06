package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class VisionIOLimelightTest {
    private static double[] botpose(int tagCount, int firstTagId) {
        double[] values = new double[11 + 7 * Math.max(tagCount, 1)];
        values[7] = tagCount;
        values[11] = firstTagId;
        return values;
    }

    @Test
    void primaryTagId_readsFirstRawFiducial() {
        assertEquals(26, VisionIOLimelight.parsePrimaryTagId(botpose(1, 26), 1));
        assertEquals(9, VisionIOLimelight.parsePrimaryTagId(botpose(2, 9), 2));
    }

    @Test
    void primaryTagId_unknownWithoutFiducials() {
        assertEquals(-1, VisionIOLimelight.parsePrimaryTagId(botpose(0, 26), 0));
        assertEquals(-1, VisionIOLimelight.parsePrimaryTagId(new double[11], 1));
    }
}
