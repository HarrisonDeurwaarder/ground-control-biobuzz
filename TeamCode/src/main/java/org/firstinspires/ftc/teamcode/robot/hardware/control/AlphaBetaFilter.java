package org.firstinspires.ftc.teamcode.robot.hardware.control;

import com.pedropathing.utils.Timer;

public class AlphaBetaFilter {
    private AlphaBetaParameters params;
    private Timer timer;

    public AlphaBetaFilter(AlphaBetaParameters parameters) {
        params = parameters;
        timer = new Timer();
    }

    public void reset() {

    }

    public static class AlphaBetaParameters {
        public double alpha, beta;

        public AlphaBetaParameters(double alpha, double beta) {
            this.alpha = alpha; this.beta = beta;
        }
    }
}
