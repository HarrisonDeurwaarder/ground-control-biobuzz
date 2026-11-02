package org.firstinspires.ftc.teamcode.lm1.math.transforms;

import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;

public interface Transform {
    public Vector apply(Vector v);
    public Vector undo(Vector v);
}
