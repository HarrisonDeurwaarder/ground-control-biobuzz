package org.firstinspires.ftc.teamcode.lm1.math.diffeq.integrators;

import org.firstinspires.ftc.teamcode.lm1.math.diffeq.ODE;
import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;

import java.util.function.BooleanSupplier;

public interface Integrator {
    public double t();
    public Vector x();
    public Vector u();
    public Vector set(double t_0, Vector x, Vector u);
    public boolean update();
}
