package org.firstinspires.ftc.teamcode.lm1.math.transforms;

import org.firstinspires.ftc.teamcode.lm1.math.linalg.Matrix;
import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;

public class Rotation extends LinearTransform {
    private final Matrix inverse;

    // a rotation is expensive to initialize, so they should only be initialized once and the same object should be reused
    public Rotation(double rx, double ry, double rz, boolean intrinsic) {
        super(build(rx, ry, rz, intrinsic));
        // for a rotation matrix, R^-1 = R^T
        inverse = Matrix.transpose(mat);
    }

    /* GETTERS */

    public Matrix inverseMatrix() { return inverse; }

    /* FUNCTIONAL METHODS */

    @Override
    public Vector undo(Vector v) { return mat.mul(v); }

    @Override
    public Vector undo(Vector v, Vector dest) { return Matrix.mul(mat, v, dest); }

    /* HELPER METHODS */
    private static Matrix build(double rx, double ry, double rz, boolean intrinsic) {
        // assembly individual rotation matrices then multiply
        // x rotation
        double cx = Math.cos(rx); double sx = Math.sin(rx);
        Matrix matX = new Matrix(new double[][]{
                {1.0, 0.0, 0.0},
                {0.0, cx,  -sx},
                {0.0, sx,   cx}
        });
        // y rotation
        double cy = Math.cos(ry); double sy = Math.sin(ry);
        Matrix matY = new Matrix(new double[][]{
                {cy,  0.0,  sy},
                {0.0, 1.0, 0.0},
                {-sy, 0.0,  cy}
        });
        // z rotation
        double cz = Math.cos(rz); double sz = Math.sin(rz);
        Matrix matZ = new Matrix(new double[][]{
                {cz,  -sz, 0.0},
                {sz,  cz,  0.0},
                {0.0, 0.0, 1.0}
        });

        // in-place methods are called to avoid reallocating new objects
        // individual matrices are discarded

        // intrinsic---around the object's own axes
        if (intrinsic) {
            return matX.mul(matY).mul(matZ);
            // extrinsic---around the world's axes
        } else {
            return matZ.mul(matY).mul(matX);
        }
    }
}
