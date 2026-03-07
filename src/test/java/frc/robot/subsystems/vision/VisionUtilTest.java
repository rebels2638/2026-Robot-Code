package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.junit.jupiter.api.Test;

class VisionUtilTest {
    @Test
    void fieldDimensionsStayAlignedWithBLineFlippingUtil() {
        assertTrue(VisionUtil.fieldDimensionsMatchFlippingUtil());
    }

    @Test
    void isPoseWithinField_acceptsInBoundsPose() {
        assertTrue(
            VisionUtil.isPoseWithinField(
                new Pose3d(
                    VisionUtil.getFieldLengthMeters() / 2.0,
                    VisionUtil.getFieldWidthMeters() / 2.0,
                    0.0,
                    new Rotation3d()
                )
            )
        );
    }

    @Test
    void isPoseWithinField_rejectsOutOfBoundsPose() {
        assertFalse(VisionUtil.isPoseWithinField(new Pose3d(-0.01, 2.0, 0.0, new Rotation3d())));
        assertFalse(
            VisionUtil.isPoseWithinField(
                new Pose3d(VisionUtil.getFieldLengthMeters() + 0.01, 2.0, 0.0, new Rotation3d())
            )
        );
        assertFalse(
            VisionUtil.isPoseWithinField(
                new Pose3d(2.0, VisionUtil.getFieldWidthMeters() + 0.01, 0.0, new Rotation3d())
            )
        );
    }
}
