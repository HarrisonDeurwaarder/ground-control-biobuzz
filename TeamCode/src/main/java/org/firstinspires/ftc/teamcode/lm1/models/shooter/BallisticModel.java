package org.firstinspires.ftc.teamcode.lm1.models.shooter;

import org.firstinspires.ftc.teamcode.lm1.math.LinearEquation;
import org.firstinspires.ftc.teamcode.lm1.models.LinearRegression;

public class BallisticModel {

    public static double                     G_IN = 386.09;
    public static double     FLYWHEEL_DIAMETER_IN = 2.83;
    public static double        MIN_ENTRY_DEGREES = 60.0;
    public static double        MAX_ENTRY_DEGREES = 85.0;
    public static double         MIN_EXIT_DEGREES = 60.0;
    public static double         MAX_EXIT_DEGREES = 85.0;

    private double minExitRadians, maxExitRadians, minEntryRadians, maxEntryRadians;

    private final LinearEquation velApx;
    public BallisticModel(LinearEquation velocityApproximator) {
        // require an approximator to map FLYWHEEL linear velocity to EXIT velocity
        velApx = velocityApproximator;
    }

    public BallisticState compute(double dx, double dy, double dz) {
        computeConstants();
        // dx on the shooting plane
        double r = Math.hypot(dx, dy);

        /* COMPUTE EXIT ANGLE */
        // tan(theta_entry) = 2h/dx - tan(theta_exit)

        // entry capability bounds defined by the hood
        double minCapableRadians = computeTheta(r, dz, minEntryRadians), maxCapableRadians = computeTheta(r, dz, maxEntryRadians);
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

        double numerator = G_IN * r * r;
        double denominator = 2 * Math.pow(Math.cos(exitTheta), 2.0) * (r * Math.tan(exitTheta) - dz);
        double exitV = Math.sqrt(numerator / denominator); // this is exit velocity of the ball
        // COMPRESSION and ENERGY TRANSFER mean ball velocity != tangential flywheel velocity
        // invert the tanVel -> exVel mapping
        double v = velApx.inverse(exitV);

        return new BallisticState(v, Math.toDegrees(exitTheta), valid);
    }

    private double computeTheta(double r, double height, double radians) { return Math.atan((2 * height / r) + Math.tan(radians)); }

    private void computeConstants() {
        // this function will be removed or deleted once angular measurements won't be subject to change
        minEntryRadians = Math.toRadians(MIN_ENTRY_DEGREES);
        maxEntryRadians = Math.toRadians(MAX_ENTRY_DEGREES);
        minExitRadians = Math.toRadians(MIN_EXIT_DEGREES);
        maxExitRadians = Math.toRadians(MAX_EXIT_DEGREES);
    }
}