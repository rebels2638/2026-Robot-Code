package frc.robot.demo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.junit.jupiter.api.Test;

class ShotCorrectionTableTest {
    // Simple stand-in model: hood 50 deg, RPS grows with distance and height.
    private static final ShotCorrectionTable.Model MODEL = (distance, height) ->
        new GentleShotModel.Solution(true, 50.0, 6.0, 20.0 + 2.0 * distance + 4.0 * height, 1.0);

    private static CalibrationPoint point(double distance, double height, double hoodOffset, double rpsOffset) {
        GentleShotModel.Solution model = MODEL.solve(distance, height);
        return new CalibrationPoint(distance, height, model.hoodDegrees() + hoodOffset, model.flywheelRps() + rpsOffset, "test");
    }

    @Test
    void noPoints_noCorrection() {
        ShotCorrectionTable table = new ShotCorrectionTable(List.of(), MODEL);
        assertEquals(ShotCorrectionTable.Correction.NONE, table.correctionAt(3.0, 1.2));
    }

    @Test
    void atSavedPoint_returnsExactlyWhatHit() {
        ShotCorrectionTable table = new ShotCorrectionTable(List.of(point(3.0, 1.2, 2.0, 0.9)), MODEL);
        ShotCorrectionTable.Correction correction = table.correctionAt(3.0, 1.2);

        assertEquals(2.0, correction.hoodDegrees(), 1e-9);
        assertEquals(0.9, correction.flywheelRps(), 1e-9);
        assertEquals(1.0, correction.strength(), 1e-9);
    }

    @Test
    void betweenTwoPoints_blendsByCloseness() {
        ShotCorrectionTable table = new ShotCorrectionTable(
            List.of(point(3.0, 1.2, 0.0, 0.9), point(4.0, 1.2, 0.0, 1.5)),
            MODEL
        );

        assertEquals(1.2, table.correctionAt(3.5, 1.2).flywheelRps(), 1e-9);
        double nearFirst = table.correctionAt(3.2, 1.2).flywheelRps();
        assertEquals(true, nearFirst > 0.9 && nearFirst < 1.2);
    }

    @Test
    void fadesAwayFromAllPoints() {
        ShotCorrectionTable table = new ShotCorrectionTable(List.of(point(3.0, 1.2, 0.0, 1.0)), MODEL);

        // 0.5 m of height counts like 1 m of distance: 0.25 m higher is still full strength.
        assertEquals(1.0, table.correctionAt(3.0, 1.45).flywheelRps(), 1e-9);
        // Normalized distance 1.0 is halfway through the fade.
        assertEquals(0.5, table.correctionAt(4.0, 1.2).flywheelRps(), 1e-9);
        // 0.8 m higher (normalized 1.6) is past the fade: model only.
        assertEquals(0.0, table.correctionAt(3.0, 2.0).flywheelRps(), 1e-9);
    }

    @Test
    void ignoresInvalidAndUnreachablePoints() {
        ShotCorrectionTable.Model unreachableAtFive = (distance, height) -> distance > 4.5
            ? GentleShotModel.Solution.infeasible(70.0)
            : MODEL.solve(distance, height);
        ShotCorrectionTable table = new ShotCorrectionTable(
            List.of(
                new CalibrationPoint(Double.NaN, 1.2, 50.0, 30.0, "bad"),
                new CalibrationPoint(5.0, 1.2, 50.0, 40.0, "unreachable")
            ),
            unreachableAtFive
        );

        assertEquals(0, table.usablePointCount());
        assertEquals(2, table.points().size());
    }
}
