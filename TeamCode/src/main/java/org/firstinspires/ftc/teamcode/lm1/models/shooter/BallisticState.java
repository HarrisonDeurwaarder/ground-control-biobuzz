package org.firstinspires.ftc.teamcode.lm1.models.shooter;

public class BallisticState {
    private double v, theta;
    private boolean valid;

    public BallisticState(double v, double theta, boolean valid) {
        this.v = v; this.theta = theta; this.valid = valid;
    }
    public BallisticState() { this(0.0, 0.0, false); }

    public void update(double v, double theta, boolean valid) { this.v = v; this.theta = theta; this.valid = valid; }

    public double v() { return v; } // inches/second
    public double omega() { return (2.0 * 180.0 * v) / (BallisticModel.FLYWHEEL_DIAMETER_IN * Math.PI); } // degrees/second
    public double theta() { return theta; } // degrees
    public boolean valid() { return valid; }
}
