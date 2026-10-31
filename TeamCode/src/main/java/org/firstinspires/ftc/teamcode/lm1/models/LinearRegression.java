package org.firstinspires.ftc.teamcode.lm1.models;

import org.firstinspires.ftc.teamcode.lm1.math.LinearEquation;
import org.firstinspires.ftc.teamcode.lm1.math.Point;

public class LinearRegression implements FunctionApproximator {

    private LinearEquation equation;

    public LinearRegression(Point... points) {

        /* ERROR HANDLING */

        // lengths > 1
        if (points.length < 2) throw new IllegalArgumentException("'points' must have two or more elements");
        // identical Xs
        double baseline = points[0].x(); boolean differenceExists = false;
        for (Point point : points) if (point.x() == baseline) {
            differenceExists = true; break;
        }
        if (!differenceExists) throw new IllegalArgumentException("'x' must have variance");

        /* FIT */

        fit(points);
    }

    public LinearEquation equation() { return equation; }

    public double infer(double x) { return equation.apply(x); }
    private void fit(Point[] points) {
        // we use the least-squares solution for linear regression

        // compute means required for deviation
        double sx = 0.0, sy = 0.0;
        for (Point point : points) { sx += point.x(); sy += point.y(); }

        double mx = sx / points.length, my = sy / points.length;

        // summations required for LS (slope)
        double sxy = 0, sxx = 0;
        for (Point point : points) {
            double deviationX = point.x() - mx;
            // least squares requires covariance and variance to compute slope
            // think of it as: however much y and x co-vary / however much x varies
            sxy += deviationX * (point.y() - my);
            sxx += Math.pow(deviationX, 2.0);
        }

        // compute SLOPE
        double slope = sxy / sxx;
        // compute INTERCEPT
        double intercept = my - slope * mx;

        equation = new LinearEquation(slope, intercept);
    }
}
