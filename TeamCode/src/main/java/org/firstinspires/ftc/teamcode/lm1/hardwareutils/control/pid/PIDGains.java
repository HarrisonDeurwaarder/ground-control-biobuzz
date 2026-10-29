package org.firstinspires.ftc.teamcode.lm1.hardwareutils.control.pid;

public class PIDGains {

    public double Kp, Ki, Kd, Kf, Ks;
    public PIDGains(double Kp, double Ki, double Kd, double Kf, double Ks) {
        this.Kp = Kp; this.Ki = Ki; this.Kd = Kd; this.Kf = Kf; this.Ks = Ks;
    }

    public PIDGains(double Kp, double Ki, double Kd, double Kf)  {
        this(Kp, Ki, Kd, Kf, 0.0);
    }

    public PIDGains(double Kp, double Ki, double Kd)  {
        this(Kp, Ki, Kd, 0.0, 0.0);
    }
}
