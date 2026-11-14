package org.firstinspires.ftc.teamcode.robot.actions;

import androidx.annotation.NonNull;

public interface Action {
    public void start();
    public boolean update();
    public boolean completed();
    public double secondsActive();
    @NonNull
    public Action clone();
}
