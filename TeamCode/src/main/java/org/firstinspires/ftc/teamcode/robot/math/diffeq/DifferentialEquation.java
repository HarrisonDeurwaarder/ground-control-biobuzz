package org.firstinspires.ftc.teamcode.robot.math.diffeq;

import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;

public interface DifferentialEquation {
    public int dimX();
    public Vector derivative(double t, Vector x, Vector dest);
    public Vector derivative(double t, Vector x);
}
