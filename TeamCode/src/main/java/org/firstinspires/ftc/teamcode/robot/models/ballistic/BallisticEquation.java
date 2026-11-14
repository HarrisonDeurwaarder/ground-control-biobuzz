package org.firstinspires.ftc.teamcode.robot.models.ballistic;

import org.firstinspires.ftc.teamcode.robot.math.diffeq.DifferentialEquation;
import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;

public class BallisticEquation implements DifferentialEquation {
    public static double G_IN = 386.09;

    private Vector u;
    private BallisticParameters params;

    // scratch vectors
    private final Vector p, v, vp, a, drag, magnus;

    public BallisticEquation(BallisticParameters parameters) {
        Vector zero2d = Vector.zero(2);
        params = parameters;
        v = zero2d.clone(); p = zero2d.clone();    vp = zero2d.clone();
        a = zero2d.clone(); drag = zero2d.clone(); magnus = zero2d.clone();
    }

    /* GETTERS */

    public int dimX() { return 2; } // p

    /* FUNCTIONAL METHODS */

    public Vector derivative(double t, Vector x, Vector dest) {
        // application doesn't depend on t

        // unpack "x" to position and velocity
        p.set(x.at(0), x.at(1));
        v.set(x.at(2), x.at(3));

        /* COMPUTE VELOCITY-DEPENDENT FORCES */

        double mag = v.mag();
        // magnus applies perpendicular to velocity
        Vector.normal(v, vp);
        // flip direction if it logically combats expected direction given spin
        if (
                params.spin == BallisticParameters.Spin.FORWARD && vp.at(1) > 0.0 ||
                params.spin == BallisticParameters.Spin.BACK    && vp.at(1) < 0.0
        ) vp.neg();

        // compute drag
        if (params.dragCoef != 0.0) drag.set(
                -params.dragCoef * mag * v.at(0),
                -params.dragCoef * mag * v.at(1)
        );

        // compute magnus
        if (params.magnusCoef != 0.0) magnus.set(
                -params.magnusCoef * mag * vp.at(0),
                -params.magnusCoef * mag * vp.at(1)
        );

        // set acceleration vector
        // acceleration aggregates grav, drag, and magnus
        a.set(0.0, -G_IN);
        a.add(drag).add(magnus);

        // acceleration is the derivative of velocity, velocity is the derivative of position
        // update output accordingly
        dest.set(v.at(0), v.at(1), a.at(0), a.at(1));
        return dest;
    }

    public Vector derivative(double t, Vector x) { return derivative(t, x, Vector.zero(4)); }

    /* HELPERS */

    public static class BallisticParameters {
        public double dragCoef, magnusCoef;
        public Spin spin;

        public enum Spin { BACK, FORWARD }

        public BallisticParameters(double dragCoefficient, double magnusCoefficient, Spin spin) {
            dragCoef = dragCoefficient; magnusCoef = magnusCoefficient; this.spin = spin;
        }

        public BallisticParameters(double dragCoefficient, double magnusCoefficient) { this(dragCoefficient, magnusCoefficient, Spin.BACK); }
        public BallisticParameters(double dragCoefficient) { this(dragCoefficient, 0.0, Spin.BACK); }
        public BallisticParameters() { this(0.0, 0.0, Spin.BACK); }
    }
}
