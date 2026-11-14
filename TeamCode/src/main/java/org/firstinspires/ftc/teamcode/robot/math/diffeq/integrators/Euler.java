package org.firstinspires.ftc.teamcode.robot.math.diffeq.integrators;

import org.firstinspires.ftc.teamcode.robot.math.diffeq.DifferentialEquation;
import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;
import static org.firstinspires.ftc.teamcode.robot.math.linalg.Vector.zero;

public class Euler implements Integrator {
    private double t;
    private Vector x;
    private DifferentialEquation eq;
    private final double dt;

    // scratch vectors
    private Vector xdt;

    public Euler(DifferentialEquation eq, double dt) {
        this.dt = dt; this.eq = eq;
        // allocate
        x = zero(eq.dimX());
        xdt = zero(eq.dimX());
    }

    /* GETTERS AND SETTERS */

    public double t() { return t; }
    public Vector x() { return x; }

    public void set(double t_0, Vector x) {
        this.t = t_0; this.x.from(x);
    }

    /* FUNCTIONAL METHODS */

    public void step() {
        // apply euler's method to the diffeq
        // small steps
        eq.derivative(t, x, xdt);
        // apply to x
        x.add(xdt.mul(dt));
        // update t
        t += dt;
    }
}
