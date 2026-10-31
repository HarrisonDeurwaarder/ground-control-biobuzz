package org.firstinspires.ftc.teamcode.lm1.models;

public class BallisticModel {

    private FunctionApproximator velApx;
    public BallisticModel(FunctionApproximator velocityApproximator) {
        // require an approximator to map FLYWHEEL linear velocity to EXIT velocity
        velApx = velocityApproximator;
    }


}
