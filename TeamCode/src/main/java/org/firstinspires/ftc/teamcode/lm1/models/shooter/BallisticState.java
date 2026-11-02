package org.firstinspires.ftc.teamcode.lm1.models.shooter;

public class BallisticState {
    private double v, exitTheta, entryTheta;
    private boolean valid;

    public BallisticState(double v, double exitTheta, double entryTheta, boolean valid) {
        this.v = v; this.exitTheta = exitTheta; this.entryTheta = entryTheta; this.valid = valid;
    }
    public BallisticState() { this(0.0, 0.0, 0.0, false); }

    /* GETTERS */

    public double v() { return v; } // inches/second
    public double omega() { return (2.0 * 180.0 * v) / (BallisticModel.FLYWHEEL_DIAMETER_IN * Math.PI); } // degrees/second
    public double exitTheta() { return exitTheta; } // degrees
    public double entryTheta() { return exitTheta; } // degrees
    public boolean valid() { return valid; }

    /* FUNCTIONAL METHODS */

    public void update(double v, double exitTheta, double entryTheta, boolean valid) {
        this.v = v; this.exitTheta = exitTheta; this.entryTheta = entryTheta; this.valid = valid;
    }
    public void updateInvalid() { update(0.0, 0.0, 0.0, false); }

    /* FACTORY */

    public static BallisticState invalid() { return new BallisticState(0.0, 0.0, 0.0, false); }
}
