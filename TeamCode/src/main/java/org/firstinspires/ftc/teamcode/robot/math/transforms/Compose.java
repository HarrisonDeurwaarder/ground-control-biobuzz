package org.firstinspires.ftc.teamcode.robot.math.transforms;

import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;

public class Compose implements Transform {
    private final Transform[] transforms;
    private Vector[] transitionalVectors; // length N+1 --- holds inputs
    private final int inF, outF;

    public Compose(Transform... transforms) {
        this.transforms = transforms;
        // grab features
        inF = transforms[0].inFeat();
        outF = transforms[transforms.length-1].outFeat();
        // confirm adjacency
        confirmMatchingDims();
        // build transient storage
        build();
    }

    /* GETTERS */

    public int inFeat() { return inF; }
    public int outFeat() { return outF; }
    public int length() { return transforms.length; }

    /* FUNCTIONAL METHODS */

    public Vector apply(Vector v, Vector dest) {
        transitionalVectors[0] = v; // copying reference is ok since [0] is only used for the next vector then overridden next call
        for (int i = 0; i < length(); i++) { transforms[i].apply(transitionalVectors[i], transitionalVectors[i+1]); }
        // copy final elements to dest
        dest.from(transitionalVectors[length() + 1]);
        return dest;
    }
    public Vector apply(Vector v) { return apply(v, Vector.zero(outF)); }

    public Vector undo(Vector v, Vector dest) {
        // reusing the same vector---can't parallelize on the same object
        transitionalVectors[length()] = v; // copying reference is ok since [0] is only used for the next vector then overridden next call
        for (int i = length(); i >= 1; i--) { transforms[i].undo(transitionalVectors[i], transitionalVectors[i-1]); }
        // copy final elements to dest
        dest.from(transitionalVectors[length() + 1]);
        return dest;
    }
    public Vector undo(Vector v) { return undo(v, Vector.zero(inF)); }

    /* HELPERS */

    public void build() {
        // instantiate
        transitionalVectors = new Vector[length() + 1];
        // iteratively build placeholders
        for (int i = 0; i < length() + 1; i++) transitionalVectors[i] = Vector.zero(transforms[i].outFeat());
    }

    public void confirmMatchingDims() {
        for (int i = 0; i < transforms.length-1; i++) {
            if (transforms[i].outFeat() != transforms[i+1].inFeat()) throw new IllegalArgumentException("Adjacent transforms must share input/output features");
        }
    }
}
