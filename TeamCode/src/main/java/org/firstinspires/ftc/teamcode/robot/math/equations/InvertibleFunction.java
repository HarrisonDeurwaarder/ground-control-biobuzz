package org.firstinspires.ftc.teamcode.robot.math.equations;

public interface InvertibleFunction extends Function {
    public double apply(double x);
    public double inverse(double y);
}
