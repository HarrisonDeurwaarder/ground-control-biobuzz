package org.firstinspires.ftc.teamcode.lm1.actions.dynamic;

import org.firstinspires.ftc.teamcode.lm1.actions.Action;

public class DynamicRace extends ActionQueue {

    public DynamicRace(boolean ephemeral, Action... actions) { super(ephemeral, actions); }

    @Override
    public boolean update() {

        // execute every action
        for (int i = 0; i < subactions.size(); i++) {
            // halt immediately once an action finishes
            if (subactions.get(i).update()) { // action is done
                // tack on removing the action if it's ephemeral
                // since we immediately break out, the incorrect indexing isn't an issue
                if (ephemeral) subactions.remove(i);
                return true;
            }
        }
        return false;
    }
}
