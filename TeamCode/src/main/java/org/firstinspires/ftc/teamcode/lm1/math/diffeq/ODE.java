package org.firstinspires.ftc.teamcode.lm1.math.diffeq;

import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;

public interface ODE {
    public Vector derivative(double t, Vector x, Vector u);
}
