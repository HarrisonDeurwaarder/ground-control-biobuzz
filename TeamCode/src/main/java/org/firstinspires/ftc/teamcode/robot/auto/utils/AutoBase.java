package org.firstinspires.ftc.teamcode.robot.auto.utils;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.actions.fixed.CompositeAction;
import org.firstinspires.ftc.teamcode.robot.actions.fixed.SequentialAction;
import org.firstinspires.ftc.teamcode.robot.hardware.HardwareController;
import org.firstinspires.ftc.teamcode.pedro.Constants;


public abstract class AutoBase extends OpMode {
    // auto timer
    protected Timer opmodeTimer;

    // pathing objects
    protected Follower follower;

    // telemetry objects
    private TelemetryManager panels;
    // auto objects
    protected HardwareController hwc;
    protected CompositeAction actions;

    protected abstract Pose getStartingPose();

    @Override
    public void init() {
        // initialize timer
        opmodeTimer = new Timer();
        opmodeTimer.reset();
        // initialize telemetry
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        // initialize and set up follower
        follower = Constants.create(hardwareMap);
        follower.setPose(getStartingPose());
        follower.update();
        // initialize custom auto objects
        hwc = new HardwareController(hardwareMap, follower, HardwareController.Team.RED);

        // construct paths to register in actions
        buildPaths();
        // create actions object
        initializeActions();
    }

    @Override
    public void init_loop() {
        // display telemetry
        addDefaultTelemetry();

        panels.update();
        telemetry.update();
    }

    @Override
    public void start() {
        // time from start not init
        opmodeTimer.reset();
        // reset auto actions
        actions.start();
    }

    @Override
    public void loop() {
        // update auto objects
        follower.update();
        boolean done = actions.update();

        // save position to blackboard in case of issues
        blackboard.put("Auto Pose", follower.pose());

        // if auto is done, halt to prevent any indexing errors
        if (done) {
            requestOpModeStop();
            // exit loop() immediately
            // requestOpModeStop() prevents any further calls, but it does not halt the current call
            return;
        }

        // display telemetry
        addDefaultTelemetry();

        panels.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        // save ending position to blackboard
        blackboard.put("Auto Pose", follower.pose());
        // disable all motors
        follower.stop();
    }

    protected abstract void buildPaths();

    protected abstract void initializeActions();

    protected void displayTelemetryMessage() {
        telemetry.addLine("An AUTONOMOUS has been selected. Press start to begin.");
    }

    /**
     * Add and publish default telemetry for auto
     */
    protected void addDefaultTelemetry() {
        // this method should be modified for game-specific data
        /* LOCALIZATION VARIABLES */
        panels.addData("Current Target Pose (IN)", follower.currentPath() != null ? follower.currentPath().get(1.0) : null);
        panels.addData("Position (IN)", follower.pose());
        panels.addData("Velocity (IN/SEC)", follower.velocity());
        panels.addData("Distance Remaining (IN)", follower.remainingDistance());
    }
}


