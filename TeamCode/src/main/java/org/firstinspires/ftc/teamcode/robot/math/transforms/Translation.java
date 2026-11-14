package org.firstinspires.ftc.teamcode.robot.math.transforms;

import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;

public class Translation implements Transform {
    private final Vector v;
    private final int inF, outF;
    public Translation(Vector v) { this.v = v; inF = v.dim(); outF = v.dim(); }

    /* GETTERS */

    public int inFeat() { return inF; }
    public int outFeat() { return outF; }

    /* FUNCTIONAL METHODS */

    public Vector apply(Vector v) { return Vector.add(this.v, v); }
    public Vector apply(Vector v, Vector dest) { return Vector.add(this.v, v, dest); }

    public Vector undo(Vector v) { return Vector.sub(this.v, v); }
    public Vector undo(Vector v, Vector dest) { return Vector.sub(this.v, v, dest); }
}
