package org.firstinspires.ftc.teamcode.robot.actions.fixed;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.robot.actions.Action;

public class SequentialAction extends CompositeAction {
    private int state;

    public SequentialAction(Action... actions) { super(actions); }

    @Override
    public void start() { super.start(); state = 0; }

    /* GETTERS */

    public int state() { return state; }

    /* FUNCTIONAL METHODS */

    @Override
    public boolean update() {
        // RETURN on an empty array
        // triggers the below condition
        if (subactions.length == 0) isCompleted = true;

        // INTERRUPT a past-completion polling parent
        if (isCompleted) return true;

        // sequential only runs ONE AT A TIME
        Action subaction = subactions[state];

        // START if marked as such
        // start happens to ONLY THE CURRENT ACTION
        // this branch should only successfully start on the FIRST ACTION
        attemptStart(state);

        /* now, EXECUTE*/
        boolean subactionCompleted = subaction.update();

        // sequential requires the CURRENT action's completion to proceed
        if (subactionCompleted) {
            completions[state] = true;
            // BRANCH: current action is complete BUT there are more
            // (checks if incrementing by 1 will reach the bound)
            // increments, then preemptively starts
            if (state+1 < subactions.length) { state++; attemptStart(state); }
            // BRANCH: current action is complete AND it's the final one
            else isCompleted = true;
        }

        return isCompleted;
    }

    @NonNull
    public SequentialAction clone() {
        // must recursively clone actions
        Action[] actionClones = new Action[length()];
        // clone all
        for (int i = 0; i < length(); i++) { actionClones[i] = subactions[i].clone(); }
        // reassemble an action by its clone
        return new SequentialAction(actionClones);
    }
}
