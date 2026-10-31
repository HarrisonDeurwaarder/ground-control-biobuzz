package org.firstinspires.ftc.teamcode.lm1.math;

public class LinearEquation implements InvertibleFunction {
    private final double slope, intercept;

    public LinearEquation(double slope, double intercept) { this.slope = slope; this.intercept = intercept; }

    public double slope() { return slope; }
    public double intercept() { return intercept; }

    public double apply(double x) { return slope * x + intercept; }
    public double inverse(double y) { return (intercept - y) / slope; }
}
