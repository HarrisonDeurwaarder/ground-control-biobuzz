package org.firstinspires.ftc.teamcode.lm1.actions.fixed;

import org.firstinspires.ftc.teamcode.lm1.actions.Action;

public class SequentialAction extends CompositeAction {
    private int state;

    public SequentialAction(Action... actions) { super(actions); }

    @Override
    public void start() { super.start(); state = 0; }

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

    public int state() { return state; }
}
