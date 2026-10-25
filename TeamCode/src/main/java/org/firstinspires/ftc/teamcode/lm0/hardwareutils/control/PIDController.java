package org.firstinspires.ftc.teamcode.lm0.hardwareutils.control;


import com.pedropathing.utils.Timer;

public class PIDController {
    /*
    * Kp = Coefficient of the proportional response to error (i.e. if error is high, increase power to compensate)
    * Ki = Coefficient of steady-state compensation (this is best understood with a longer explanation; if desired, look it up)
    * Kd = Coefficient of damping; if the change in proportional response is too high, it weakens it to prevent oscillation
    * Kf = Feedforward coefficient; scales the output based on the raw setpoint magnitude (if higher setpoints will require more force); should NOT be used for position-based systems
    * Ks = Static constant; applies a constant small power to overcome friction and increase the snappiness of commands
    * Kg = (constant) Gravity constant; applies a consistent gravity-counteracting force; should be computed
    */
    public double Kp, Ki, Kd, Kf, Ks, Kg;
    public double lastMeasurement, lastTimestamp, integral, dFilter, setpoint = 0.0;
    public double staticTolerance = 1.0;
    public double tauD            = 0.0;
    public double clampI          = Integer.MAX_VALUE;
    public double IZone           = Integer.MAX_VALUE;
    private Timer plantTimer;
    private boolean first = true; // first flag to prevent exploding derivative values

    public void setStaticTolerance(double staticTolerance) { this.staticTolerance = staticTolerance; }

    public void setTauD(double tauD) { this.tauD = tauD; }

    public void setClampI(double clampI) { this.clampI = clampI; }

    public void setIZone(double IZone) { this.IZone = IZone; }

    public PIDController(double p, double i, double d, double f, double s, double g) {
        setCoefficients(p, i, d, f, s, g);
        init(staticTolerance, tauD, clampI, IZone);
    }

    private void init(double staticTolerance, double tauD, double clampI, double IZone) {
        plantTimer = new Timer();
        plantTimer.reset();

        setStaticTolerance(staticTolerance);
        setTauD(tauD);
        setClampI(clampI);
        setIZone(IZone);
    }

    @Override
    public String toString() {
        return String.format("PIDController(p=%g, i=%g, d=%g, f=%g, s=%g, error=%g)", Kp, Ki, Kd, Kf, Ks, lastMeasurement);
    }

    /**
     * Set the coefficients of the controller
     *
     * @param p proportional coefficient
     * @param i integral coefficient
     * @param d derivative coefficient
     * @param f feedforward coefficient
     * @param s static coefficient
     */
    public void setCoefficients(double p, double i, double d, double f, double s, double g) {
        // map coefficients
        Kp = p; Ki = i; Kd = d; Kf = f; Ks = s; Kg = g;
    }

    public void setCoefficients(double p, double i, double d, double s, double g) {
        // map coefficients
        Kp = p; Ki = i; Kd = d; Kf = 0.0; Ks = s; Kg = g;
    }

    public void reset() {
        first = true;
        // reset running quantities
        lastMeasurement = 0.0;
        lastTimestamp = plantTimer.seconds();
        integral = 0.0;
        dFilter = 0.0;
    }

    public void changeSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double compute(double measurement, double feedforward) {
        double timestamp = plantTimer.seconds();
        double deltaTime = timestamp - lastTimestamp;
        lastTimestamp = timestamp;
        // clip dt
        if (deltaTime <= 1e-6) deltaTime = 1e-6;

        // compute terms
        double error = setpoint - measurement;
        // only compute proportional term on first iteration
        if (first) {
            first = false;
            return Kp * error;
        }

        // clamp integral
        integral += Math.min(Math.max(-clampI, error * deltaTime), clampI);
        // zero if within tolerance
        if (Math.abs(error) <= IZone) { integral = 0.0; }

        // compute and apply a filter
        double derivative = -(measurement - lastMeasurement) / deltaTime;
        dFilter += (derivative - dFilter) * deltaTime / (tauD + deltaTime);

        // smooth static term
        double static_ = Math.tanh(error);

        lastMeasurement = measurement;

        return Kp * error + Ki * integral + Kd * derivative + Kf * feedforward + Ks * static_;
    }

    public double compute(double measurement) {
        return compute(measurement, Kf);
    }
}