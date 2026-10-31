package org.firstinspires.ftc.teamcode.lm1.actions.dynamic;

import org.firstinspires.ftc.teamcode.lm1.actions.Action;

import java.util.ArrayList;

public class ActionQueue {
    private final ArrayList<EnrichedAction> actions;
    private final boolean ephemeral;

    public ActionQueue(boolean ephemeral, Action... actions) {
        this.ephemeral = ephemeral;

        this.actions = new ArrayList<>();
        // iteratively assemble EnrichedActions and place
        // EnrichedActions encapsulate both 'start' and 'completed'
        for (Action action : actions) { this.actions.add(new EnrichedAction(action)); }
    }

    public boolean update() {
        // EXECUTE *EVERY* ACTION INDEPENDENTLY OF OTHERS
        // BRANCH: ephemeral relies on List.removeIf
        if (ephemeral) {
            actions.removeIf(a -> {
                if (!a.started()) a.start(); // START if not started
                return a.update(); // no need for a "completed => skip" condition
            });
            // all completed = empty list
            return actions.isEmpty();
        }
        // BRANCH: non-ephemeral relies on manual loop
        else {
            boolean allCompleted = true;
            for (EnrichedAction action : actions) {
                // START if marked as such
                if (!action.started()) { action.start(); }
                // SKIP if action is already complete (as is stored in memory)
                if (action.completed()) continue;

                /* now EXECUTE */
                boolean actionCompleted = action.update();
                if (!actionCompleted) allCompleted = false;
            }
            return allCompleted;
        }
    }

    public void add(Action action) { actions.add(new EnrichedAction(action)); }

    // WRAPPER FOR ACTIONS TO CONTAIN STATUS INFORMATION
    private static class EnrichedAction implements Action {
        private final Action action;
        private boolean started;

        /* CONSTRUCTORS */

        public EnrichedAction(Action action) {
            this.action = action; started = false;
        }

        /* GETTERS AND CONTRACT METHODS */

        @Override public void start() { action.start(); started = true; }
        @Override public boolean update() { return action.update(); }
        public boolean started() { return started; }
        @Override public boolean completed() { return action.completed(); }
        @Override public double secondsActive() { return action.secondsActive(); }
    }
}
