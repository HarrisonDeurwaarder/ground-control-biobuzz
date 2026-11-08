package org.firstinspires.ftc.teamcode.lm1.models.shooter;

import android.view.inputmethod.ExtractedText;

public class BallisticState {
    private double exitVelocity, tangentialVelocity, exitTheta, entryTheta, error;
    private boolean valid;

    public BallisticState(double exitVelocity, double tangentialVelocity, double exitTheta, double entryTheta, double error, boolean valid) {
        update(exitVelocity, tangentialVelocity, exitTheta, entryTheta, error, valid);
    }
    public BallisticState() {
        updateInvalid();
    }

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
        this.exitVelocity = exitVelocity; this.tangentialVelocity = tangentialVelocity;
        this.exitTheta = exitTheta; this.entryTheta = entryTheta;
        this.error = error; this.valid = valid;
    }
    public void update(BallisticState state) {
        exitVelocity = state.exitVelocity; tangentialVelocity = state.tangentialVelocity;
        exitVelocity = state.exitVelocity; entryTheta = state.entryTheta;
        error = state.error; valid = state.valid;
    }
    public void updateInvalid() { update(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.POSITIVE_INFINITY, false); }

    /* FACTORY */

    public static BallisticState invalid() { return new BallisticState(); }
}
