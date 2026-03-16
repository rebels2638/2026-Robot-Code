package frc.robot.commands.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import org.junit.jupiter.api.Test;

class AutosTest {
    private static final Pose2d TEST_RESET_POSE = new Pose2d(
        new Translation2d(1.5, 2.0),
        Rotation2d.fromDegrees(30.0)
    );

    @Test
    void shouldInsertSimResetPose_onlyInSimWhenPoseIsProvided() {
        assertTrue(Autos.shouldInsertSimResetPose(TEST_RESET_POSE, Constants.Mode.SIM));
        assertFalse(Autos.shouldInsertSimResetPose(TEST_RESET_POSE, Constants.Mode.COMP));
        assertFalse(Autos.shouldInsertSimResetPose(null, Constants.Mode.SIM));
    }

    @Test
    void resolveSimResetPose_returnsOriginalPoseWhenNoFlipIsNeeded() {
        assertEquals(TEST_RESET_POSE, Autos.resolveSimResetPose(TEST_RESET_POSE, false));
    }

    @Test
    void resolveSimResetPose_returnsFlippedPoseWhenAllianceFlipIsNeeded() {
        Pose2d flippedPose = Autos.resolveSimResetPose(TEST_RESET_POSE, true);

        assertEquals(16.54 - TEST_RESET_POSE.getX(), flippedPose.getX(), 1e-9);
        assertEquals(8.07 - TEST_RESET_POSE.getY(), flippedPose.getY(), 1e-9);
        assertEquals(-150.0, flippedPose.getRotation().getDegrees(), 1e-9);
    }

    @Test
    void resolveSimResetPose_rejectsNullPose() {
        assertThrows(IllegalArgumentException.class, () -> Autos.resolveSimResetPose(null, false));
    }
}
