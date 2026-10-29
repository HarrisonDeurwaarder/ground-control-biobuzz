package org.firstinspires.ftc.teamcode.lm1.hardwareutils.control;

public class LowPassFilter {
    double alpha;
    double state;
    boolean firstPass;

    public LowPassFilter(double alpha) {
        this.alpha = alpha;
        firstPass = true;
    }

    public double apply(double measurement) {
        // on first pass, set to measurement
        if (firstPass) {
            state = measurement;
            firstPass = false;
            // low-pass of two identical measurements yields the same measurement
            return measurement;
        }

        // maintain a running state as a measurement-smoothing mechanism
        state = alpha * measurement + (1.0 - alpha) * state;
        return state;
    }
}
