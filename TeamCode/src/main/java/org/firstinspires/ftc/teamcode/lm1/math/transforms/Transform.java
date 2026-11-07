package org.firstinspires.ftc.teamcode.lm1.math.transforms;

import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;

public interface Transform {
    public int inFeat();
    public int outFeat();
    public Vector apply(Vector v);
    public Vector apply(Vector v, Vector dest);
    public Vector undo(Vector v);
    public Vector undo(Vector v, Vector dest);
}
