package org.firstinspires.ftc.teamcode.robot.math.equations;

import org.firstinspires.ftc.teamcode.robot.math.linalg.Point;
import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;

public class LinearEquation implements InvertibleFunction {
    private final double slope, intercept;

    public LinearEquation(double slope, double intercept) { this.slope = slope; this.intercept = intercept; }

    /* GETTERS AND SHALLOW FUNCTIONAL METHODS */

    public double slope() { return slope; }
    public double intercept() { return intercept; }
    public double root() {return -intercept / slope; }

    public double apply(double x) { return slope * x + intercept; }
    public double inverse(double y) { return (intercept - y) / slope; }

    /* FACTORY */

    public static LinearEquation fromXY(double x1, double y1, double x2, double y2) {
        double slope = (y2 - y1) / (x2 - x1);
        double intercept = y1 - slope * x1;
        return new LinearEquation(slope, intercept);
    }

    public static LinearEquation fromPoints(Point p1, Point p2) { return fromXY(p1.x(), p1.y(), p2.x(), p2.y()); }

    public static LinearEquation fromVectors(Vector v1, Vector v2) {
        if (v1.dim() != 2 || v2.dim() != 2) throw new IllegalArgumentException("Vector construction of a linear equation requires 'v1' and 'v2' to have exactly 2 dimensions");
        return fromXY(v1.at(0), v1.at(1), v2.at(0), v2.at(1));
    }
}
