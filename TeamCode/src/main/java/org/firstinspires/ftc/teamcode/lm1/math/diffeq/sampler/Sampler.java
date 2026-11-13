package org.firstinspires.ftc.teamcode.lm1.math.diffeq.sampler;

public interface Sampler {
    public double lo();
    public double hi();
    public double loE();
    public double hiE();
    public boolean failed();
    public void logError(double mostRecentSampleError);
    public void reset();
    public double sample();
}
