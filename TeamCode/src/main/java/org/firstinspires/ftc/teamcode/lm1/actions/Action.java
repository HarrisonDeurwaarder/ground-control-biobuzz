package org.firstinspires.ftc.teamcode.lm1.actions;

public interface Action {
    public void start();
    public boolean update();
    public boolean completed();
    public double secondsActive();
}
