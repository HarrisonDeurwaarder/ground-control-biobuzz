package org.firstinspires.ftc.teamcode.lm1.actions.fixed;

import org.firstinspires.ftc.teamcode.lm1.actions.Action;

public class RaceAction extends CompositeAction {

    public RaceAction(Action... actions) { super(actions); }

    @Override
    public boolean update() {
        // RETURN on an empty array
        // triggers the below condition
        if (subactions.length == 0) isCompleted = true;

        // INTERRUPT a past-completion polling parent
        if (isCompleted) return true;

        // EXECUTE *EVERY* ACTION
        // but HALT on first completion
        for (int i = 0; i < subactions.length; i++) {
            // START if marked as such
            attemptStart(i);
            // SKIP if subaction is already complete (as is stored in memory)
            if (completions[i]) continue;

            /* now, EXECUTE */
            // this current action is DONE
            if (subactions[i].update()) {
                // race only requires ONE completion
                isCompleted = true;
                // update COMPLETIONS accordingly
                // race will only indicate ONE completion
                completions[i] = true;
                // exit loop
                break;
            }
        }
        return isCompleted;
    }
}
