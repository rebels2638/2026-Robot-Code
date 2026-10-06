package frc.robot.demo;

/** The demo's two ways to fire. */
public enum DemoMode {
    /** Fixed hood, flywheel and robot-relative turret angle from the dashboard. */
    MANUAL,
    /** Track the AprilTag (or a saved/placed target) and solve a gentle shot to it. */
    TAG;

    public DemoMode toggled() {
        return this == MANUAL ? TAG : MANUAL;
    }
}
