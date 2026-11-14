package org.firstinspires.ftc.teamcode.robot.math.diffeq.integrators;

import org.firstinspires.ftc.teamcode.robot.math.diffeq.DifferentialEquation;
import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;
import static org.firstinspires.ftc.teamcode.robot.math.linalg.Vector.zero;

public class RK4 implements Integrator {
    private double t;
    private Vector x;
    private DifferentialEquation eq;
    private final double dt;

    // scratch vectors
    private Vector k1, k2, k3, k4, tmp;

    public RK4(DifferentialEquation eq, double dt) {
        this.dt = dt; this.eq = eq;
        // allocate
        x = zero(eq.dimX());
        k1 = zero(eq.dimX());
        k2 = zero(eq.dimX());
        k3 = zero(eq.dimX());
        k4 = zero(eq.dimX());
        tmp = zero(eq.dimX());
    }

    /* GETTERS AND SETTERS */

    public double t() { return t; }
    public Vector x() { return x; }

    public void set(double t_0, Vector x) {
        this.t = t_0; this.x.from(x);
    }

    /* FUNCTIONAL METHODS */

    public void step() {
        // take the derivative for k1
        eq.derivative(t, x, k1);
        // for k2, a midpoint
        eq.derivative(t + dt/2.0, Vector.mul(k1, dt/2.0, tmp).add(x), k2);
        // for k3, a better midpoint
        eq.derivative(t + dt/2.0, Vector.mul(k2, dt/2.0, tmp).add(x), k3);
        // for k4, the endpoint
        eq.derivative(t + dt, Vector.mul(k3, dt, tmp).add(x), k4);

        // compute weighted sum
        k1.add(k2.mul(2.0)).add(k3.mul(2.0)).add(k4);
        // return the final x value
        x.add(k1.mul(dt/6.0));
        // update t
        t += dt;
    }
}
