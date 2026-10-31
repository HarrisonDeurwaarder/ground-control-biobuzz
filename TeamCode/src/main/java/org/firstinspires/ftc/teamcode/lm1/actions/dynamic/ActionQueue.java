package org.firstinspires.ftc.teamcode.lm1.actions.dynamic;

import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.lm1.actions.Action;

import java.util.ArrayList;
import java.util.Arrays;

public class ActionQueue {
    private ArrayList<Action> subactions;
    private Timer timer;
    private boolean ephemeral;

    public ActionQueue(boolean discardOnCompletion, Action... actions) {
        ephemeral = discardOnCompletion;
        subactions = new ArrayList<>(Arrays.asList(actions));
        // instantiate timer ONCE
        timer = new Timer();
        // ActionQueue doesn't track completions/starts
    }

    public ActionQueue(Action... actions) { this(false, actions); }

    public void start() { timer.reset(); }

    public boolean update() {

        ArrayList<Action> markedForRemoval = new ArrayList<>();

        // execute every action
        boolean allCompleted = true;

        for (int i = 0; i < subactions.size(); i++) {
            Action subaction = subactions.get(i);
            // start if needed
            if (!subaction.started()) subaction.start();

            // then, update the completion array
            boolean subactionCompleted = subaction.update();

            // mark off full completions if one is missing
            if (!subactionCompleted) allCompleted = false;
            // mark completed action for removal
            else markedForRemoval.add(subaction);
        }

        // remove entries
        subactions.removeAll(markedForRemoval);

        // update accordingly
        return allCompleted;
    }

    public ArrayList<Action> subactions() { return subactions; }
    public double secondsActive() { return timer.seconds(); }
    public int length() { return subactions.size(); }
}