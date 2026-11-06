package org.firstinspires.ftc.teamcode.lm1.models.shooter;

public class BallisticState {
    private double exitVelocity, tangentialVelocity, exitTheta, entryTheta, error;
    private boolean valid;

    public BallisticState(double exitVelocity, double tangentialVelocity, double exitTheta, double entryTheta, double error, boolean valid) {
        this.exitVelocity = exitVelocity; this.tangentialVelocity = tangentialVelocity; this.exitTheta = exitTheta; this.entryTheta = entryTheta; this.error = error; this.valid = valid;
    }
    public BallisticState() { this(0.0, 0.0, 0.0, 0.0, 0.0, false); }

    /* GETTERS */

    public double exitVelocity() { return exitVelocity; } // inches/second
    public double tangentialVelocity() { return exitVelocity; } // inches/second
    public double omega() { return (2.0 * 180.0 * exitVelocity) / (BallisticModel.FLYWHEEL_DIAMETER_IN * Math.PI); } // degrees/second
    public double exitTheta() { return exitTheta; } // degrees
    public double entryTheta() { return exitTheta; } // degrees
    public double error() { return error; } // inches/second
    public boolean valid() { return valid; }

    /* FUNCTIONAL METHODS */

    public void update(double exitVelocity, double tangentialVelocity, double exitTheta, double entryTheta, double error, boolean valid) {
        this.exitVelocity = exitVelocity; this.tangentialVelocity = tangentialVelocity; this.exitTheta = exitTheta; this.entryTheta = entryTheta; this.error = error; this.valid = valid;
    }
    public void updateInvalid() { update(0.0, 0.0, 0.0, 0.0, 0.0, false); }
    public void mirror(BallisticState other) { update(other.exitVelocity, other.tangentialVelocity, other.exitTheta, other.entryTheta, other.error, other.valid); }

    /* FACTORY */

    public static BallisticState invalid() { return new BallisticState(0.0, 0.0, 0.0, 0.0, 0.0, false); }
}
