package org.firstinspires.ftc.teamcode.lm0.hardwareutils.control;


import com.pedropathing.utils.Timer;

public class KalmanFilter {
    double lastState;
    double lastStateVariance;

    double measurementVariance;
    double processVariance;
    double b;

    double lastTimestamp = 0.0;
    Timer timer = new Timer();

    boolean isRate;

    public KalmanFilter(double initialState, double initialUncertainty, double measurementVariance, double processVariance, boolean usingRate, double b) {
        this.lastState = initialState;
        this.lastStateVariance = initialUncertainty;

        this.measurementVariance = measurementVariance;
        this.processVariance = processVariance;
        isRate = usingRate;
        this.b = b;
    }

    public KalmanFilter(double initialState, double initialUncertainty, double measurementCovariance, double processCovariance, boolean usingRate) { this(initialState, initialUncertainty, measurementCovariance, processCovariance, usingRate, 0.0); }

    public double apply(double measurement, double input) {
        // reset timer on first apply() call
        if (lastTimestamp == 0.0) {
            timer.reset();
        }
        double dt = timer.seconds() - lastTimestamp;
        lastTimestamp = timer.seconds(); // i do notttt care that they're diff
        // compute model predictions
        double predictedState = lastState + b * (isRate ? input * dt : input);
        double predictedStateVariance = lastStateVariance + processVariance * dt;

        double gain = predictedStateVariance / (predictedStateVariance + measurementVariance);
        // update the estimated state values
        lastState = predictedState + gain * (measurement - predictedState);
        lastStateVariance = (1.0 - gain) * predictedStateVariance;

        return lastState;
    }

    public double apply(double measurement) { return apply(measurement, 0.0); }
}
