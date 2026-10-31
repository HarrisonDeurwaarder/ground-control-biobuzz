package org.firstinspires.ftc.teamcode.lm1.models;

public class LinearInterpolation implements FunctionApproximator {

    private Point[] points;
    private double[] slopes, intercepts;

    public LinearInterpolation(Point... points) {
        this.points = points.clone();

        /* ERROR HANDLING */

        // length > 1
        if (points.length < 2) throw new IllegalArgumentException("'points' must have two or more elements");
        // X is in STRICTLY ASCENDING order
        for (int i = 0; i < points.length - 1; i++) {
            if (points[i].x() >= points[i+1].x()) throw new IllegalArgumentException("'Point.x' must be in ascending order");
        }

        /* FIT */

        fit();
    }

    public double[] slopes() { return slopes.clone(); }
    public double[] intercepts() { return intercepts.clone(); }
    public double rangeX() { return points[points.length - 1].x() - points[0].x(); }
    public double rangeY() {
        // compute min/max
        double min = Integer.MAX_VALUE, max = Integer.MIN_VALUE;
        for (Point point : points) {
            if (point.y() < min) min = point.y(); if (point.y() > max) max = point.y();
        }

        return max - min;
    }

    public double infer(double x) {

        /* REGULAR CASES */
        // interpolation---yes yes this is what it's meant for

        // iterate through every pair until the CORRECT BUCKET is found
        for (int i = 0; i < points.length - 1; i++) {

            // redundant if they're all ordered; but its READABLE
            if (x >= points[i].x() && x < points[i+1].x()) {
                return slopes[i] * x + intercepts[i];
            }
        }

        /* EDGE CASES */
        // extrapolation---please do not rely on this for our robot

        // below MINIMUM X
        if (x < points[0].x()) {
            return slopes[0] * x + intercepts[0];
        // above MAXIMUM X
        } else {
            return slopes[slopes.length - 1] * x + intercepts[slopes.length - 1];
        }
    }

    private void fit() {
        // instantiate empty coefficients
        slopes = new double[points.length - 1]; intercepts = new double[points.length - 1];

        for (int i = 0; i < points.length - 1; i++) {
            // compute slope
            double slope = (points[i+1].y() - points[i].y()) / (points[i+1].x() - points[i].x());
            slopes[i] = slope;
            // compute intercept
            intercepts[i] = points[i].y() - slope * points[i].x();
        }
    }
}
