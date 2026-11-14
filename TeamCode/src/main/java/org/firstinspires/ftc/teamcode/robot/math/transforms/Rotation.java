package org.firstinspires.ftc.teamcode.robot.math.transforms;

import org.firstinspires.ftc.teamcode.robot.math.linalg.Matrix;
import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;

public class Rotation extends LinearTransform {
    private final Matrix inverse;
    private double rx, ry, rz;
    private boolean intrinsic;
    private boolean needsRebuilt;
    private Matrix matx = Matrix.zero(3, 3), maty = Matrix.zero(3, 3), matz = Matrix.zero(3, 3);

    // a rotation is expensive to initialize, so they should only be initialized once and the same object should be reused
    public Rotation(double rx, double ry, double rz, boolean intrinsic) {
        // send to constructor with placeholders
        super(Matrix.zero(3, 3));
        inverse = Matrix.zero(3, 3);
        // then ACTUALLY build it
        this.rx = rx; this.ry = ry; this.rz = rz; this.intrinsic = intrinsic;
        build();
    }

    /* GETTERS & SETTERS */

    public Matrix inverseMatrix() { return inverse; }

    public void setRx(double rx) { this.rx = rx; needsRebuilt = true; }
    public void setRy(double ry) { this.ry = ry; needsRebuilt = true; }
    public void setRz(double rz) { this.rz = rz; needsRebuilt = true; }

    /* FUNCTIONAL METHODS */

    @Override
    public Vector apply(Vector v) { if (needsRebuilt) build(); return super.apply(v); }

    @Override
    public Vector apply(Vector v, Vector dest) { if (needsRebuilt) build(); return super.apply(v, dest); }

    @Override
    public Vector undo(Vector v) { if (needsRebuilt) build(); return mat.mul(v); }

    @Override
    public Vector undo(Vector v, Vector dest) { if (needsRebuilt) build(); return Matrix.mul(mat, v, dest); }

    /* HELPER METHODS */
    private void build() {
        // assembly individual rotation matrices then multiply
        // x rotation
        double cx = Math.cos(rx); double sx = Math.sin(rx);
        matx.set(1.0, 0.0, 0.0, 0.0, cx, -sx, 0.0, sx, cx);
        // y rotation
        double cy = Math.cos(ry); double sy = Math.sin(ry);
        maty.set(cy, 0.0, sy, 0.0, 1.0, 0.0, -sy, 0.0, cy);
        // z rotation
        double cz = Math.cos(rz); double sz = Math.sin(rz);
        matz.set(cz, -sz, 0.0, sz, cz, 0.0, 0.0, 0.0, 1.0);

        // in-place methods are called to avoid reallocating new objects
        // individual matrices are discarded

        // intrinsic---around the object's own axes
        if (intrinsic) {
            mat.from(matx.mul(maty).mul(matz));
            // extrinsic---around the world's axes
        } else {
            mat.from(matz.mul(maty).mul(matx));
        }
        // for a rotation matrix, R^-1 = R^T
        Matrix.transpose(mat, inverse);

        needsRebuilt = false;
    }
}
