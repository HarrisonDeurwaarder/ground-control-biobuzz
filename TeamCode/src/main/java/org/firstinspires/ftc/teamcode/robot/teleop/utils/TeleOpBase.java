package org.firstinspires.ftc.teamcode.robot.teleop.utils;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.actions.dynamic.ActionQueue;
import org.firstinspires.ftc.teamcode.robot.hardware.HardwareController;


public abstract class TeleOpBase extends OpMode {
    // teleop timer
    protected Timer opmodeTimer;

    // hardware and scheduling objects
    protected HardwareController hwc;
    protected ActionQueue queue;

    // telemetry objects
    private TelemetryManager panels;

    // teleop poses
    protected Pose recalibratedPose; // find a suitable location on the field to recalibrate odometry if necessary

    protected abstract Pose getStartingPose();

    @Override
    public void init() {
        // initialize timer
        opmodeTimer = new Timer();
        opmodeTimer.reset();
        // initialize telemetry
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        // initialize custom teleop objects
        hwc = new HardwareController(hardwareMap, HardwareController.Team.RED);
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
        // update scheduling
        queue.update();
        setTeleOpDrive();
        updateControls();

        // display telemetry
        displayControls();
        addDefaultTelemetry();

        panels.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        // stop the drivetrain (hwc is null if init failed)
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

    }

    abstract protected void updateControls(); // this should be overriden with non-driving teleop controls (i.e. intake, macros, etc)

    abstract protected void displayControls(); // this should be overriden with driver hub controls

    /**
     * Add and publish default telemetry for teleop
     */
    protected void addDefaultTelemetry() {
        // this method should be modified for game-specific data
        panels.addData("Using Default Start", !blackboard.containsKey("Auto Pose"));
    }
}


