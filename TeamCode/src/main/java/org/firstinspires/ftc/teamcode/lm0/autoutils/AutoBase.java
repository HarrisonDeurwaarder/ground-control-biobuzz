package org.firstinspires.ftc.teamcode.lm0.autoutils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareController;
import org.firstinspires.ftc.teamcode.pedro.Constants;


public abstract class AutoBase extends OpMode {
    // auto timer
    protected Timer opmodeTimer;
    // pathing objects
    protected Follower follower;
    protected HardwareController hardwareController;
    protected AutoBuilder autoBuilder;

    // telemetry objects
    protected TelemetryPacket packet;
    protected FtcDashboard dashboard;

    // these poses will always be present
    protected static Pose defaultEndPose; // auto intends to end here, but pose will be tracked regardless and saved to teleop
    protected abstract Pose getStartingPose();

    @Override
    public void init() {
        // initialize timer
        opmodeTimer = new Timer();
        opmodeTimer.reset();
        // initialize telemetry
        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        // initialize and set up follower
        follower = Constants.create(hardwareMap);
        follower.setPose(getStartingPose());
        follower.update();
        // initialize custom auto objects
        hardwareController = new HardwareController(hardwareMap, follower, HardwareController.Team.RED);
        autoBuilder = new AutoBuilder(follower);

        // construct paths to register in AutoBuilder
        buildPaths();
        // add cycles to AutoBuilder
        registerPaths();
    }

    @Override
    public void init_loop() {
        // display telemetry
        addDefaultTelemetry();
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void start() {
        // time from start not init
        opmodeTimer.reset();
        // reset AutoBuilder
        autoBuilder.reset();
    }

    @Override
    public void loop() {
        // update auto objects
        follower.update();
        boolean done = autoBuilder.update();

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
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        // save ending position to blackboard
        blackboard.put("Auto Pose", follower.pose());
        // disable all motors
        follower.stop();
    }

    protected abstract void buildPaths();

    protected abstract void registerPaths();

    protected void displayTelemetryMessage() {
        telemetry.addLine("An AUTONOMOUS has been selected. Press start to begin.");
        telemetry.update();
    }

    /**
     * Add and publish default telemetry for auto
     */
    protected void addDefaultTelemetry() {
        // this method should be modified for game-specific data
        // pathing variables
        packet.put("Auto Length", autoBuilder.getAutoLength());
        packet.put("Cycle State", autoBuilder.getCycleState());
        packet.put("Cycle Timer", autoBuilder.getCycleTime());
        packet.put("Cycle Length", autoBuilder.getCurrentCycle().getCycleLength());
        packet.put("Path State", autoBuilder.getCurrentCycle().getPathState());
        packet.put("Path Timer", autoBuilder.getCurrentCycle().getPathTime());
        // localization variables
        packet.put("Current Target Pose", follower.currentPath() != null ? follower.currentPath().get(1.0) : null); // find the pose the at end (t=1.0) of the parametric curve
        packet.put("Position", follower.pose());
        packet.put("Velocity", follower.velocity());
        packet.put("Distance Remaining", follower.remainingDistance());
    }
}


