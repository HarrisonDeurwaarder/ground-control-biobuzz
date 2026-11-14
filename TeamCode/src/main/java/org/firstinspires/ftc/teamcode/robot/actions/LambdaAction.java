package org.firstinspires.ftc.teamcode.robot.actions;

import androidx.annotation.NonNull;

import com.pedropathing.utils.Timer;

import java.util.function.BooleanSupplier;

public class LambdaAction implements Action {
    private Runnable onStart, onStep, onStop;
    private BooleanSupplier completionCondition;
    private Timer timer;
    private boolean isCompleted = false; // mark completion and disable bookkeeping if programs run without resetting

    public LambdaAction(Runnable onStart, Runnable onStep, Runnable onStop, BooleanSupplier completionCondition) {
        this.onStart = onStart; this.onStep = onStep; this.onStop = onStop ;this.completionCondition = completionCondition;
        // instantiate timer
        timer = new Timer();
    }

    /* GETTERS */

    @Override public boolean completed() { return isCompleted; }
    @Override public double secondsActive() { return timer.seconds(); }

    /* FUNCTIONAL METHODS */

    @Override
    public void start() {
        timer.reset();
        // only now indicate noncompletion
        isCompleted = false;
        onStart.run();
    }

    @Override
    public boolean update() {
        // INTERRUPT a past-end polling parent
        if (isCompleted) return true;

        // EXECUTE action content
        onStep.run();
        // CHECK completion
        isCompleted = completionCondition.getAsBoolean();
        // run stop if completed
        if (isCompleted) onStop.run();
        return isCompleted;
    }

    @NonNull
    public LambdaAction clone() { return new LambdaAction(onStart, onStep, onStop, completionCondition); }
}
