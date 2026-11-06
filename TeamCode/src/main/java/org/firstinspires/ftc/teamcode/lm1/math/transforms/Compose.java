package org.firstinspires.ftc.teamcode.lm1.math.transforms;

import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;

public class Compose implements Transform {
    private final Transform[] transforms;
    public Compose(Transform... transforms) { this.transforms = transforms; }

    /* FUNCTIONAL METHODS */

    public Vector apply(Vector v) {
        Vector vector = v.clone();
        for (Transform t : transforms) { vector = t.apply(vector); }
        return vector;
    }

    public Vector undo(Vector v) {
        // attempt to undo
        // if any unsupported transforms are added, they will throw
        Vector vector = v.clone();
        for (int i = transforms.length - 1; i >= 0; i--) { vector = transforms[i].undo(vector); }
        return vector;
    }
}
