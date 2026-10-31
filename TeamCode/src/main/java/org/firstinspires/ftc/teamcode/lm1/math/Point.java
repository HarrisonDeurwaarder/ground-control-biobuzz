package org.firstinspires.ftc.teamcode.lm1.math;

public class Point {
    private final double x, y;
    private Point(double x, double y) {
        this.x = x; this.y = y;
    }

    public double x() { return x; }
    public double y() { return y; }

    /* STATIC FACTORY */
    public static Point point(double x, double y) { return new Point(x, y); }
}
