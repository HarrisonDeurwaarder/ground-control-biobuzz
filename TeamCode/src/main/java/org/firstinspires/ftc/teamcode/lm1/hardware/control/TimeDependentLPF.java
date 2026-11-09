package org.firstinspires.ftc.teamcode.lm1.hardware.control;

import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;

public class TimeDependentLPF {
    private Vector state;
    private Vector a1, a2, o, divtau, va; // applications write in-place instead of allocating a new vector
    private boolean firstPass;
    private Timer timer;

    // variants
    private Vector vectorTau;
    private double scalarTau;
    private final boolean usingScalarTau; // cumulative alpha for all elements vs. vector alpha

    public TimeDependentLPF(Vector tau) {
        this.vectorTau = tau;
        usingScalarTau = false;
        reset();

        // make preallocations
        int d = tau.dim();
        a1 = Vector.zero(d); a2 = Vector.zero(d); o = Vector.zero(d); divtau = Vector.zero(d); o = Vector.zero(d);
    }
    public TimeDependentLPF(double tau, int d) {
        this.scalarTau = tau;
        usingScalarTau = true;
        reset();
    }

    public Vector apply(Vector measurement) {
        // policy is the same on first pass
        if (firstPass) {
            firstPass = false;
            // low-pass of state=measurement and measurement -> measurement
            state = measurement.clone();
            return measurement;
        }

        double dt = timer.seconds();
        // then apply either scalar or vector alpha
        if (usingScalarTau) {
            double scalarAlpha = Math.exp(dt / scalarTau);
            Vector.mul(measurement, scalarAlpha, a1);
            Vector.mul(state, 1.0 - scalarAlpha, a2);
            Vector.add(a1, a2, o);
        } else {
            // get alpha
            Vector.pow(vectorTau, -1.0, divtau);
            Vector.mul(divtau, dt, divtau);
            Vector.exp(divtau, vectorTau);
            // term 2
            // (using vtau as complement)
            Vector.mul(state, vectorTau, a2);
            // term 1
            // (using vtau as non-complement)
            Vector.sub(vectorTau, 1.0, vectorTau);
            Vector.mul(measurement, vectorTau, a1);
            Vector.add(a1, a2, o);
        }

        timer.reset();
        return o;
    }

    public void reset() { firstPass = true; timer.reset(); }
}