package org.firstinspires.ftc.teamcode.lm1.models;

public class LinearRegression implements FunctionApproximator {

    private double slope, intercept;

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

    public double slope() { return slope; }
    public double intercept() { return intercept; }

    public double infer(double x) { return slope * x + intercept; }

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
        slope = sxy / sxx;
        // compute INTERCEPT
        intercept = my - slope * mx;
    }
}
