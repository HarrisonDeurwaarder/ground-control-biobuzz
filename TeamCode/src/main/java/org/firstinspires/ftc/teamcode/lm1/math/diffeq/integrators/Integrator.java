package org.firstinspires.ftc.teamcode.lm1.math.diffeq.integrators;

import org.firstinspires.ftc.teamcode.lm1.math.diffeq.DifferentialEquation;
import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;

public interface Integrator {
    public double t();
    public Vector x();
    public void set(double t_0, Vector x);
    public void step();
}
