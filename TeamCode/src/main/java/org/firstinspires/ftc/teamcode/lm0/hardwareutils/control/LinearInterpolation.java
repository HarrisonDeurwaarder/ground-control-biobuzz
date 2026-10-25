package org.firstinspires.ftc.teamcode.lm0.hardwareutils.control;

import java.util.ArrayList;

public class LinearInterpolation {

    private ArrayList<Double> xValues = new ArrayList<Double>();
    private ArrayList<Double> yValues = new ArrayList<Double>();
    private ArrayList<Double> slopes = new ArrayList<Double>();
    private ArrayList<Double> intercepts = new ArrayList<Double>();
    private boolean hasBeenComputed = false;

    public void addPoint(double x, double y) {
        // get the most recent element
        // throw if it's less than the last
        if (x < xValues.get(xValues.size() - 1)) {
            throw new IllegalArgumentException("Expected x to be the greatest present (>=" + xValues.get(xValues.size() - 1).toString() + "); got " + x);
        }
        xValues.add(x);
        yValues.add(x);
    }

    public void computeCoefficients() {
        // iterate through every sequential point pair
        for (int i = 0; i < xValues.size() - 1; i++) {
            // compute slope by estimating average rate of change
            double slope = (yValues.get(i + 1) - yValues.get(i)) / (xValues.get(i + 1) - xValues.get(i));
            // add to ArrayLists
            slopes.add(slope);
            intercepts.add(
                    yValues.get(i) - slope * xValues.get(i)
            );
        }
        // enable use of inference method
        hasBeenComputed = true;
    }

    public double infer(double x) {
        // throw if the points haven't been compiled
        if (!hasBeenComputed) {
            throw new IllegalStateException("computeCoefficients() must be called before infer()");
        }
        // iterate through every pair until the correct range is found
        for (int i = 0; i < xValues.size() - 1; i++) {
            // redundant if they're all ordered; but its readable
            if (x >= xValues.get(i) && x < xValues.get(i + 1)) {
                return slopes.get(i) * x + intercepts.get(i);
            }
        }
        // edge cases
        // only run if the loop didn't find any matches
        if (x < xValues.get(0)) { // if the value is LESS than the minimum
            return slopes.get(0) * x + intercepts.get(0);
        } else { // if the value is MORE than the
            return slopes.get(slopes.size() - 1) * x + intercepts.get(intercepts.size() - 1);
        }
    }
}
