package org.firstinspires.ftc.teamcode.robot.math.diffeq.integrators;

import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;

public interface Integrator {
    public double t();
    public Vector x();
    public void set(double t_0, Vector x);
    public void step();
}
