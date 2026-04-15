package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class SuperstructureAlternatingIntakeTest {
    @Test
    void resolveNextAlternatingIntakeTarget_startsAtFirst() {
        assertEquals(
            Superstructure.AlternatingIntakeTarget.FIRST,
            Superstructure.resolveNextAlternatingIntakeTarget(null, false, false)
        );
    }

    @Test
    void resolveNextAlternatingIntakeTarget_holdsTargetUntilPivotSettles() {
        assertEquals(
            Superstructure.AlternatingIntakeTarget.FIRST,
            Superstructure.resolveNextAlternatingIntakeTarget(
                Superstructure.AlternatingIntakeTarget.FIRST,
                false,
                false
            )
        );
        assertEquals(
            Superstructure.AlternatingIntakeTarget.SECOND,
            Superstructure.resolveNextAlternatingIntakeTarget(
                Superstructure.AlternatingIntakeTarget.SECOND,
                false,
                false
            )
        );
    }

    @Test
    void resolveNextAlternatingIntakeTarget_flipsFromFirstToSecondAtSetpoint() {
        assertEquals(
            Superstructure.AlternatingIntakeTarget.SECOND,
            Superstructure.resolveNextAlternatingIntakeTarget(
                Superstructure.AlternatingIntakeTarget.FIRST,
                true,
                false
            )
        );
    }

    @Test
    void resolveNextAlternatingIntakeTarget_flipsFromSecondToFirstAtSetpoint() {
        assertEquals(
            Superstructure.AlternatingIntakeTarget.FIRST,
            Superstructure.resolveNextAlternatingIntakeTarget(
                Superstructure.AlternatingIntakeTarget.SECOND,
                true,
                false
            )
        );
    }

    @Test
    void resolveNextAlternatingIntakeTarget_flipsOnTimeout() {
        assertEquals(
            Superstructure.AlternatingIntakeTarget.SECOND,
            Superstructure.resolveNextAlternatingIntakeTarget(
                Superstructure.AlternatingIntakeTarget.FIRST,
                false,
                true
            )
        );
        assertEquals(
            Superstructure.AlternatingIntakeTarget.FIRST,
            Superstructure.resolveNextAlternatingIntakeTarget(
                Superstructure.AlternatingIntakeTarget.SECOND,
                false,
                true
            )
        );
    }
}
