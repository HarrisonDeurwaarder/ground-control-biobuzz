package org.firstinspires.ftc.teamcode.robot.models.shooter;

import org.firstinspires.ftc.teamcode.robot.math.equations.LinearEquation;

public class BallisticModel {

    public static double                     G_IN = 386.09;
    public static double     FLYWHEEL_DIAMETER_IN = 2.83;
    public static double        MIN_ENTRY_DEGREES = 60.0;
    public static double        MAX_ENTRY_DEGREES = 85.0;
    public static double         MIN_EXIT_DEGREES = 60.0;
    public static double         MAX_EXIT_DEGREES = 85.0;

    protected double minExitRadians, maxExitRadians, minEntryRadians, maxEntryRadians;

    protected final BallisticState state = new BallisticState();
    protected final LinearEquation velApx;
    public BallisticModel(LinearEquation velocityApproximator) {
        // require an approximator to map FLYWHEEL linear velocity to EXIT velocity
        velApx = velocityApproximator;
    }

    /* GETTERS */

    public BallisticState state() { return state; }

    /* FUNCTIONAL METHODS */

    public void compute(double dx, double dy, double dz) {
        computeConversions();
        // dx on the shooting plane
        double r = Math.hypot(dx, dy);

        /* COMPUTE EXIT ANGLE */
        // tan(theta_entry) = 2h/dx - tan(theta_exit)

        // entry capability bounds defined by the hood
        double minCapableRadians = computeHoodTheta(r, dz, minEntryRadians), maxCapableRadians = computeHoodTheta(r, dz, maxEntryRadians);
        // theta* is the ideal theta value where flywheel velocity is minimized
        double thetaStar = Math.toRadians(45.0) + Math.atan2(dz, r) / 2.0;

        // clip theta to satify HOOD CONSTRAINTS and prevent DIV BY ZERO
        // select theta* if possible
        double exitTheta; boolean valid;

        // OUT-OF-BOUNDS CASES
        // here, we must snap to the best available option (no perfect solution)
        if (maxCapableRadians < minExitRadians) { exitTheta = maxCapableRadians; valid = false; }
        else if (minCapableRadians > maxExitRadians) { exitTheta = minCapableRadians; valid = false; }

        // IN-BOUNDS CASES
        else {
            // we bound the intersection (knowing they overlap)
            // try theta* else return something in-bounds
            double lo = Math.max(minCapableRadians, minExitRadians), hi = Math.min(maxCapableRadians, maxExitRadians);

            // try theta*, then default to minimum
            exitTheta = Math.max(lo, Math.min(thetaStar, hi));
            valid = true;
        }

        /* COMPUTE EXIT VELOCITY */

        double exitV = computeExitV(r, dz, exitTheta);
        // COMPRESSION and ENERGY TRANSFER mean ball velocity != tangential flywheel velocity
        // invert the tanVel -> exVel mapping
        double v = velApx.inverse(exitV);

        //state.update(v, exitTheta, valid);
    }

    protected double computeHoodTheta(double r, double height, double entryRadians) { return Math.atan((2 * height / r) + Math.tan(entryRadians)); }

    protected double computeExitV(double r, double height, double exitRadians) {
        double numerator = G_IN * r * r;
        double denominator = 2 * Math.pow(Math.cos(exitRadians), 2.0) * (r * Math.tan(exitRadians) - height);
        return Math.sqrt(numerator / denominator); // this is exit velocity of the ball
    }

    protected void computeConversions() {
        // this function will be removed or deleted once angular measurements won't be subject to change
        minEntryRadians = Math.toRadians(MIN_ENTRY_DEGREES);
        maxEntryRadians = Math.toRadians(MAX_ENTRY_DEGREES);
        minExitRadians = Math.toRadians(MIN_EXIT_DEGREES);
        maxExitRadians = Math.toRadians(MAX_EXIT_DEGREES);
    }
}