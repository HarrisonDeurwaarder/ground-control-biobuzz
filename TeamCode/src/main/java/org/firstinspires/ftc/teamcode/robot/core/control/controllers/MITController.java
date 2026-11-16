package org.firstinspires.ftc.teamcode.robot.core.control.controllers;

import org.firstinspires.ftc.teamcode.robot.MissionControl;

public class MITController {
    private MITParameters params;
    private MissionControl.Task task;
    private MissionControl misCtrl = MissionControl.instance();

    /* CONSTRUCTORS */

    public MITController(MITParameters parameters, boolean autobind) {
        params = parameters;
        // bind to MisCtrl
        task = new MissionControl.Task(this::update);
        if (autobind) MissionControl.instance().bind(task);
    }

    /* FUNCTIONAL METHODS */

    public void update() {
        // get dt for scaling

    }

    /* NESTED CLASSES */

    public static class MITParameters {
        public double kP, kD, ff;

        public MITParameters(double kP, double kD, double ff) {
            this.kP = kP; this.kD = kD; this.ff = ff;
        }
    }
}
