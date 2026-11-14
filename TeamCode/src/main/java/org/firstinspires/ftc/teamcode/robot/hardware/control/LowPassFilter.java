package org.firstinspires.ftc.teamcode.robot.hardware.control;

import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;

public class LowPassFilter {
    private Vector state;
    private Vector a1, a2, o; // applications write in-place instead of allocating a new vector
    private boolean firstPass;

    // variants
    private Vector vectorAlpha, vectorComplement; // alpha => how much to trust the incoming measurement
    private double scalarAlpha;
    private final boolean usingScalarAlpha; // cumulative alpha for all elements vs. vector alpha

    public LowPassFilter(Vector alpha) {
        this.vectorAlpha = alpha;
        vectorComplement = Vector.sub(Vector.one(alpha.dim()), alpha);
        usingScalarAlpha = false;
        reset();

        // make preallocations
        int d = alpha.dim();
        a1 = Vector.zero(d); a2 = Vector.zero(d); o = Vector.zero(d);
    }
    public LowPassFilter(double alpha, int d) {
        this.scalarAlpha = alpha;
        usingScalarAlpha = true;
        reset();

        // make preallocations
        a1 = Vector.zero(d); a2 = Vector.zero(d); o = Vector.zero(d);
    }

    public Vector apply(Vector measurement) {
        // policy is the same on first pass
        if (firstPass) {
            firstPass = false;
            // low-pass of state=measurement and measurement -> measurement
            state = measurement.clone();
            return measurement;
        }
        // then apply either scalar or vector alpha
        if (usingScalarAlpha) {
            Vector.mul(measurement, scalarAlpha, a1);
            Vector.mul(state, 1.0 - scalarAlpha, a2);
            Vector.add(a1, a2, o);
        } else {
            Vector.mul(measurement, vectorAlpha, a1);
            Vector.mul(state, vectorComplement, a2);
            Vector.add(a1, a2, o);
        }
        return o;
    }

    public void reset() { firstPass = true; }
}
