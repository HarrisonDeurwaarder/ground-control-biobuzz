package org.firstinspires.ftc.teamcode.lm1.math.transforms;

import org.firstinspires.ftc.teamcode.lm1.math.linalg.Matrix;
import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;

public class LinearTransform implements Transform {
    protected final Matrix mat;
    public LinearTransform(Matrix mat) { this.mat = mat; }

    /* GETTERS */

    public Matrix matrix() { return mat; }

    /* FUNCTIONAL METHODS */

    public Vector apply(Vector v) { return mat.mul(v); }
    public Vector undo(Vector v) { throw new UnsupportedOperationException("Undo transformation is not yet implemented"); }
}
