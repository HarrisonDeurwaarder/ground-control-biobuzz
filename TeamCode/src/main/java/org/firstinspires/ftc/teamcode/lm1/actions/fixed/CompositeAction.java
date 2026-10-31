package org.firstinspires.ftc.teamcode.lm1.actions.fixed;

import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.lm1.actions.Action;

import java.util.Arrays;

abstract public class CompositeAction implements Action {
    // three subaction tracking arrays for 1) objects 2) completions 3) starts
    protected Action[] subactions;
    protected boolean[] completions;
    protected boolean[] starts;

    protected Timer timer;
    protected boolean isCompleted = false;

    public CompositeAction(Action... actions) {
        subactions = actions.clone();
        // instantiate timer ONCE
        timer = new Timer();
        // track completions & starts for all subactions
        completions = new boolean[actions.length];
        starts = new boolean[actions.length];
    }

    @Override
    public void start() {
        // reset timer TO TRACK CURRENT
        timer.reset();
        // reset completion
        isCompleted = false;
        // reset arrays
        Arrays.fill(completions, false);
        Arrays.fill(starts, false);
    }

    public Action[] subactions() { return subactions.clone(); }
    @Override public double secondsActive() { return timer.seconds(); }
    @Override public boolean completed() { return isCompleted; }
    public boolean[] completions() { return completions.clone(); }
    public boolean[] starts() { return starts.clone(); }
    public int length() { return subactions.length; }

    protected void attemptStart(int i) {
        if (!starts[i]) {
            subactions[i].start();
            starts[i] = true; // mark as started
            // action is already marked as INCOMPLETE
        };
    }
}