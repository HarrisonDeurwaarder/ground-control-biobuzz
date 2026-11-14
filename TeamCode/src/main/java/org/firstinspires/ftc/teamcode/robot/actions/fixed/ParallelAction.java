package org.firstinspires.ftc.teamcode.robot.actions.fixed;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.robot.actions.Action;

public class ParallelAction extends CompositeAction {

    public ParallelAction(Action... actions) { super(actions); }

    @Override
    public boolean update() {
        // RETURN on an empty array
        // triggers the below condition
        if (subactions.length == 0) isCompleted = true;

        // INTERRUPT a past-completion polling parent
        if (isCompleted) return true;

        // EXECUTE *EVERY* ACTION
        boolean allCompleted = true;
        for (int i = 0; i < subactions.length; i++) {
            // START if marked as such
            attemptStart(i);
            // SKIP if subaction is already complete (as is stored in memory)
            if (completions[i]) continue;

            /* now EXECUTE */
            // then, update the completion array
            boolean subactionCompleted = subactions[i].update();
            // update completions
            // this will never reset a TRUE completion alone (it will have previously been skipped)
            completions[i] = subactionCompleted;

            // separately, track if every action has been completed
            // this will only run if the action hasn't PREVIOUSLY been completed
            // AND it hasn't completed this iteration
            if (!subactionCompleted) allCompleted = false;
        }

        // update accordingly
        // again, this will not override an already-true isCompleted
        isCompleted = allCompleted;

        return isCompleted;
    }

    @NonNull
    public ParallelAction clone() {
        // must recursively clone actions
        Action[] actionClones = new Action[length()];
        // clone all
        for (int i = 0; i < length(); i++) { actionClones[i] = subactions[i].clone(); }
        // reassemble an action by its clone
        return new ParallelAction(actionClones);
    }
}
