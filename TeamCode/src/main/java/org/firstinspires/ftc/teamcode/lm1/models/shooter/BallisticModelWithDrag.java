package org.firstinspires.ftc.teamcode.lm1.models.shooter;

import org.firstinspires.ftc.teamcode.lm1.math.equations.LinearEquation;
import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;
import static org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector.*;

public class BallisticModelWithDrag extends BallisticModel {
    public static final double[] EXIT_ANGLE_CANDIDATES = {45.0, 50.0, 55.0, 60.0}; // degrees
    public static final double[] EXIT_VELOCITY_CANDIDATES = {10.0, 20.0}; // in/sec
    public static final double INIT_EXIT_VEL = 10.0; // in/sec
    public static final double DT = 0.1;
    public static final int MAX_SOLVER_ITERATIONS = 100;
    public static final double MAXIMUM_ENTRY_ANGLE = 0.0;

    private static final double DRAG_COEFFICIENT = 1.0;

    public enum RootFinder { BISECT, SECANT, REGULA_FALSI };

    private RootFinder rootFinder;
    private double[] exitRadianCandidates;

    public BallisticModelWithDrag(LinearEquation velocityApproximator, RootFinder rootFinder) { super(velocityApproximator); this.rootFinder = rootFinder; }
    public BallisticModelWithDrag(LinearEquation velocityApproximator) { this(velocityApproximator, RootFinder.REGULA_FALSI); }

    /* FUNCTIONAL METHODS */

    public void compute(double dx, double dy, double dz) {
        computeConstants();
        // dx on the shooting plane
        double r = Math.hypot(dx, dy);
        // compute the drag constant

        // ballistics lacks a closed-form model with drag
        // hence, we must apply Euler's method to solve the ODE

        // candidate state vector to compare the entry angles
        BallisticState[] candidateStates = new BallisticState[EXIT_ANGLE_CANDIDATES.length];

        for (int i = 0; i < EXIT_ANGLE_CANDIDATES.length; i++) {
            double candidateExitTheta = EXIT_ANGLE_CANDIDATES[i];

            // we allow a discrete set of entry angles and optimize for each
            // then, we select the best one
            // candidate history to inform choosing the next candidate
            Candidate[] candidateVBuffer = {new Candidate(0.0), new Candidate(0.0)};
            for (int k = 0; k < EXIT_VELOCITY_CANDIDATES.length; k++) candidateVBuffer[k] = new Candidate(EXIT_VELOCITY_CANDIDATES[k]);

            // track the current x value of the shooter model
            // p is position in the coordinate plane pointing for the shooter to the goal + verticality
            // origin begins from the launch point of the ball
            Vector p = zero(2);
            //Vector v = vector(candidateVBuffer[1] * Math.sin(candidateExitTheta), candidateVBuffer[1] * Math.cos(candidateExitTheta));
            Vector a = vector(0.0, -G_IN); // starting acceleration: zero in X, -g in Y
            // two exit conditions
            // 1) the model converges to p.x=r
            // 2) the model fails to converge in reasonable time
            int j = 0;
            while (true) {
                // compute new acceleration (apply the drag force)
                //a = a.sub(
                        //v.mul(v.mag() * DRAG_COEFFICIENT * DT)
                //);
                // apply acceleration to velocity and velocity to position
                //v = v.add(a.mul(DT)); p = p.add(v.mul(DT));

                /* CHECK CONVERGENCE */

                // compute entry angle
                //double entryTheta = Math.atan2(v.at(0), v.at(1));
                // if the ball reaches distance AND entry angle < max (required by hive geometry)
                // then log the state
                //if (p.at(0) >= r && entryTheta < MAXIMUM_ENTRY_ANGLE) {
                    //candidateStates[i] = new BallisticState(
                            //candidateVQueue[1].value, candidateExitTheta, entryTheta, true
                    //);
                //}
                // increment in place of a for-loop
                j++;
                // end with an invalid result if model fails
                if (j > MAX_SOLVER_ITERATIONS) {
                    candidateStates[i] = BallisticState.invalid(); break;
                }
            }
        }
    }

    @Override
    protected void computeConstants() {
        super.computeConstants();
        // tack on the angle array
        exitRadianCandidates = new double[EXIT_ANGLE_CANDIDATES.length];
        for (int i = 0; i < EXIT_ANGLE_CANDIDATES.length; i++) exitRadianCandidates[i] = Math.toRadians(EXIT_ANGLE_CANDIDATES[i]);
    }

    private Candidate[] selectNextCandidate(Candidate[] buffer, int i) {
        double candidateValue;
        // use preset candidate values before generating new ones
        if (i < EXIT_VELOCITY_CANDIDATES.length) candidateValue = EXIT_VELOCITY_CANDIDATES[i];
        // else, rely on history and generate new ones
        else {
            // query method by enum value
            if (rootFinder == RootFinder.BISECT) { candidateValue = bisect(buffer); }
            else if (rootFinder == RootFinder.SECANT) { candidateValue = secant(buffer); }
            else { throw new IllegalArgumentException("'rootFinder' must be a RootFinder"); }
        }

        // wrap value in Candidate
        Candidate candidate = new Candidate(candidateValue);
        // update buffer
        Candidate[] newBuffer = { buffer[1], candidate };
        return newBuffer;
    }

    /* ROOT FINDERS */
    private static double bisect(Candidate[] buffer) {
        // bisect the two guesses
        // this method is guaranteed to converge
        // however, it does so slowly because it's agnostic to how big the miss is
        return (buffer[0].value + buffer[1].value) / 2.0;
    }

    private static double secant(Candidate[] candidates) {
        // compute a linear approximation of the model using two guesses
        // compute the root of that linear equation as the next guess
        LinearEquation lineq = LinearEquation.fromXY(
                candidates[0].value, candidates[0].error, candidates[1].value, candidates[1].error
        );
        return lineq.root();
    }

    static class Candidate {
        double value, error;
        public Candidate(double value) { this.value = value; }

        /* GETTERS & SETTERS */
        public double value() { return value; }
        public double error() { return error; }
        public void setError(double error) { this.error = error; }
    }
}