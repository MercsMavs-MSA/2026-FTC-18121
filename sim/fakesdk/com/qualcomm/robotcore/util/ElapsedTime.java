package com.qualcomm.robotcore.util;

/** Runs on the simulated clock, so timing in the robot code is deterministic. */
public class ElapsedTime {
    private long startNs = sim.SimWorld.nowNanos();
    public void reset() { startNs = sim.SimWorld.nowNanos(); }
    public double nanoseconds() { return sim.SimWorld.nowNanos() - startNs; }
    public double milliseconds() { return nanoseconds() / 1e6; }
    public double seconds() { return nanoseconds() / 1e9; }
}
