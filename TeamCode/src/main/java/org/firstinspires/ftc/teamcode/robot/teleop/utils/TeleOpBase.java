package org.firstinspires.ftc.teamcode.robot.teleop.utils;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.actions.dynamic.ActionQueue;
import org.firstinspires.ftc.teamcode.robot.hardware.HardwareController;
import org.firstinspires.ftc.teamcode.pedro.Constants;


public abstract class TeleOpBase extends OpMode {
    // teleop timer
    protected Timer opmodeTimer;

    // pathing objects
    protected Follower follower;
    protected HardwareController hwc;
    protected ActionQueue queue;

    // telemetry objects
    private TelemetryManager panels;

    // teleop poses
    protected Pose defaultStartingPose;
    protected Pose recalibratedPose; // find a suitable location on the field to recalibrate odometry if necessary

    @Override
    public void init() {
        // initialize timer
        opmodeTimer = new Timer();
        opmodeTimer.reset();
        // initialize telemetry
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        // initialize and set up follower
        follower = Constants.create(hardwareMap);
        follower.setPose(
                // it is suggested to use blackboard for other aspects of auto that will impact teleop if auto fails
                (Pose) blackboard.getOrDefault("Auto Pose", defaultStartingPose) // if auto fails, ensure teleop can recover correct odometry
        );
        // initialize custom teleop objects
        hwc = new HardwareController(hardwareMap, follower, HardwareController.Team.RED);
        // action queue for scheduling parallel actions
        queue = new ActionQueue();
    }

    @Override
    public void init_loop() {
        // display telemetry
        displayControls();
        addDefaultTelemetry();

        panels.update();
        telemetry.update();
    }

    @Override
    public void loop() {
        // update pathing and scheduling
        queue.update();
        follower.update();
        setTeleOpDrive();

        // display telemetry
        displayControls();
        addDefaultTelemetry();

        panels.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        // disable following
        follower.stop();
    }

    protected double[] getDriveControls() {
        // front-facing field-oriented controls will differ by team
        // if this doesn't work, experiment and find a configuration that does
        double[] controls = {
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
        };
        return controls;
    }

    protected final void setTeleOpDrive() {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
    }

    abstract protected void updateControls(); // this should be overriden with non-driving teleop controls (i.e. intake, macros, etc)

    abstract protected void displayControls(); // this should be overriden with driver hub controls

    /**
     * Add and publish default telemetry for teleop
     */
    protected void addDefaultTelemetry() {
        // this method should be modified for game-specific data
        panels.addData("Position (IN)", follower.pose());
        panels.addData("Velocity (IN/SEC)", follower.velocity());
        panels.addData("Using Default Start", !blackboard.containsKey("Auto Pose"));
    }
}


