package org.firstinspires.ftc.teamcode.lm1.actions.dynamic;

import org.firstinspires.ftc.teamcode.lm1.actions.Action;

public class DynamicSequential extends ActionQueue {
    private int state;

    public DynamicSequential(boolean ephemeral, Action... actions) { super(ephemeral, actions); }

    @Override
    public void start() { super.start(); state = 0; }

    @Override
    public boolean update() {

        boolean subactionCompleted;
        /* EPHEMERAL BRANCH */
        // state is unused in the ephemeral branch
        if (ephemeral) {
            subactionCompleted = subactions.get(0).update();
            if (subactionCompleted) subactions.remove(0);
            // emptiness = completion
            return subactions.isEmpty();

        /* PERSISTENT BRANCH */
        } else {
            // exit if ioob
            if (atEnd()) return true;

            // execute action
            subactionCompleted = subactions.get(state).update();
            // ADVANCE if SUB-ACTION is done
            if (subactionCompleted) state += 1;
            return subactionCompleted;
        }
    }

    public int state() { return state; }

    public boolean atEnd() { return state >= subactions.size(); }
}
