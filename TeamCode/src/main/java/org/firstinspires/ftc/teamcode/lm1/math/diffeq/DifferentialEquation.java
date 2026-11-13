package org.firstinspires.ftc.teamcode.lm1.math.diffeq;

import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;

public interface DifferentialEquation {
    public int dimX();
    public Vector derivative(double t, Vector x, Vector dest);
    public Vector derivative(double t, Vector x);
}
