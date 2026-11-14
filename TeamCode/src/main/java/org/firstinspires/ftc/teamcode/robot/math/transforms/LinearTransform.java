package org.firstinspires.ftc.teamcode.robot.math.transforms;

import org.firstinspires.ftc.teamcode.robot.math.linalg.Matrix;
import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;

public class LinearTransform implements Transform {
    protected final Matrix mat;
    protected final int inF, outF;
    public LinearTransform(Matrix mat) { this.mat = mat; inF = mat.cols(); outF = mat.rows();}

    /* GETTERS */

    public Matrix matrix() { return mat; }
    public int inFeat() { return inF; }
    public int outFeat() { return outF; }

    /* FUNCTIONAL METHODS */

    public Vector apply(Vector v) { return mat.mul(v); }
    public Vector apply(Vector v, Vector dest) { return Matrix.mul(mat, v, dest); }

    public Vector undo(Vector v) { throw new UnsupportedOperationException("Undo transformation is not yet implemented"); }
    public Vector undo(Vector v, Vector dest) { throw new UnsupportedOperationException("Undo transformation is not yet implemented"); }
}
