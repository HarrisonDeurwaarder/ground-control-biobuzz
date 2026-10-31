package org.firstinspires.ftc.teamcode.lm1.actions.dynamic;

import org.firstinspires.ftc.teamcode.lm1.actions.Action;

public class DynamicParallel extends ActionQueue {

    public DynamicParallel(boolean ephemeral, Action... actions) { super(ephemeral, actions); }

    @Override
    public boolean update() {

        /* EPHEMERAL BRANCH */
        // relies on removeIf and separately counting completions in the predicate
        if (ephemeral) {
            subactions.removeIf(Action::update);
            // if the ArrayList is empty, all actions must be complete
            return subactions.isEmpty();

        /* PERSISTENT BRANCH */
        } else {
            // execute every action
            // THIS LOGIC DOESN'T CHANGE WITH AN ARRAYLIST (minus absence of completions)
            boolean allCompleted = true;
            for (int i = 0; i < subactions.size(); i++) {
                // only run if subaction is incomplete
                // then, update the completion array
                boolean subactionCompleted = subactions.get(i).update();

                // mark off completions
                if (!subactionCompleted) allCompleted = false;
            }

            // update accordingly
            return allCompleted;
        }
    }
}
