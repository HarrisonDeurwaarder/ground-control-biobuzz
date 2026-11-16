package org.firstinspires.ftc.teamcode.robot.utils;

import org.firstinspires.ftc.teamcode.robot.MissionControl;

import java.util.concurrent.TimeUnit;

public class Timer {
    private long resetTimestamp;

    private MissionControl misCtrl = MissionControl.instance();

    /* CONSTRUCTORS */

    public Timer() { reset(); }

    /* GETTERS */

    public long nanos() { return misCtrl.time() - resetTimestamp; }
    public long millis() { return TimeUnit.NANOSECONDS.toMillis(misCtrl.time() - resetTimestamp); }
    public double seconds() { return (misCtrl.time() - resetTimestamp) / 1e9; }

    /* FUNCTIONAL METHODS */

    public void reset() { resetTimestamp = misCtrl.time(); }
}
