package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.util.ShotCalculator.ShotData;
import org.junit.jupiter.api.Test;

class SuperstructureShotReadinessTest {
    @Test
    // Ready when both impact error and distance bounds are satisfied.
    void isShotReady_trueWhenImpactAndDistanceAreValid() {
        boolean ready = Superstructure.isShotReady(0.05, 0.3, 4.0, 2.0, 6.0);
        assertTrue(ready);
    }

    @Test
    // Exceeding impact tolerance should fail readiness.
    void isShotReady_falseWhenImpactExceedsTolerance() {
        boolean ready = Superstructure.isShotReady(0.31, 0.3, 4.0, 2.0, 6.0);
        assertFalse(ready);
    }

    @Test
    // Distances below configured minimum should fail readiness.
    void isShotReady_falseWhenDistanceBelowMinimum() {
        boolean ready = Superstructure.isShotReady(0.05, 0.3, 1.99, 2.0, 6.0);
        assertFalse(ready);
    }

    @Test
    // Distances above configured maximum should fail readiness.
    void isShotReady_falseWhenDistanceAboveMaximum() {
        boolean ready = Superstructure.isShotReady(0.05, 0.3, 6.01, 2.0, 6.0);
        assertFalse(ready);
    }

    @Test
    // Min/max boundaries are inclusive.
    void isShotReady_allowsBoundaryDistances() {
        assertTrue(Superstructure.isShotReady(0.05, 0.3, 2.0, 2.0, 6.0));
        assertTrue(Superstructure.isShotReady(0.05, 0.3, 6.0, 2.0, 6.0));
    }

    @Test
    void selectShotDataWithMinDistanceGuard_usesLastInRangeWhenTooClose() {
        ShotData mostRecentTooClose = shotData(1.5);
        ShotData lastInRange = shotData(2.5);

        ShotData selected = Superstructure.selectShotDataWithMinDistanceGuard(
            mostRecentTooClose,
            lastInRange,
            2.0
        );

        assertSame(lastInRange, selected);
    }

    @Test
    void selectShotDataWithMinDistanceGuard_fallsBackToMostRecentWhenNoLastInRange() {
        ShotData mostRecentTooClose = shotData(1.5);

        ShotData selected = Superstructure.selectShotDataWithMinDistanceGuard(
            mostRecentTooClose,
            null,
            2.0
        );

        assertSame(mostRecentTooClose, selected);
    }

    @Test
    void selectShotDataWithMinDistanceGuard_usesMostRecentWhenNotTooClose() {
        ShotData mostRecentOutOfRangeFar = shotData(7.0);
        ShotData lastInRange = shotData(4.0);

        ShotData selected = Superstructure.selectShotDataWithMinDistanceGuard(
            mostRecentOutOfRangeFar,
            lastInRange,
            2.0
        );

        assertSame(mostRecentOutOfRangeFar, selected);
    }

    @Test
    void isLastInRangeShotDataValid_falseWhenTimestampIsNotFinite() {
        assertFalse(Superstructure.isLastInRangeShotDataValid(10.0, Double.NaN, 1.0));
        assertFalse(Superstructure.isLastInRangeShotDataValid(10.0, Double.NEGATIVE_INFINITY, 1.0));
    }

    @Test
    void isLastInRangeShotDataValid_trueWhenShotAgeAtOrBelowMaxAge() {
        assertTrue(Superstructure.isLastInRangeShotDataValid(10.0, 9.0, 1.0));
        assertTrue(Superstructure.isLastInRangeShotDataValid(10.0, 9.1, 1.0));
    }

    @Test
    void isLastInRangeShotDataValid_falseWhenShotAgeExceedsMaxAge() {
        assertFalse(Superstructure.isLastInRangeShotDataValid(10.0, 8.9, 1.0));
    }

    private static ShotData shotData(double effectiveDistanceMeters) {
        return new ShotData(
            Rotation2d.fromDegrees(0.0),
            Rotation2d.fromDegrees(0.0),
            0.0,
            0.0,
            0.0,
            effectiveDistanceMeters,
            new Translation2d()
        );
    }
}
