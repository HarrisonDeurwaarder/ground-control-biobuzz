package org.firstinspires.ftc.teamcode.lm1.math.transforms;

import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;

public class Translation implements Transform {
    private final Vector v;
    public Translation(Vector v) { this.v = v; }

    /* FUNCTIONAL METHODS */

    public Vector apply(Vector v) { return v.add(this.v); }
    public Vector undo(Vector v) { return v.sub(this.v); }
}
