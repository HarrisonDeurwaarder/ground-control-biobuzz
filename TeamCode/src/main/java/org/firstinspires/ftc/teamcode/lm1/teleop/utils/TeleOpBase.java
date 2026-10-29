package org.firstinspires.ftc.teamcode.lm1.teleop.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.lm1.hardwareutils.HardwareController;
import org.firstinspires.ftc.teamcode.pedro.Constants;


public abstract class TeleOpBase extends OpMode {
    // teleop timer
    protected Timer opmodeTimer;
    // pathing objects
    protected Follower follower;
    protected HardwareController hwc;

    // telemetry objects
    protected TelemetryPacket packet;
    protected FtcDashboard dashboard;

    // teleop poses
    protected Pose defaultStartingPose;
    protected Pose recalibratedPose; // find a suitable location on the field to recalibrate odometry if necessary

    // boolean flags editable by dashboard
    public static boolean robotCentric = true;

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
        follower.setPose(
                // it is suggested to use blackboard for other aspects of auto that will impact teleop if auto fails
                (Pose) blackboard.getOrDefault("Auto Pose", defaultStartingPose) // if auto fails, ensure teleop can recover correct odometry
        );
        // initialize custom auto objects
        hwc = new HardwareController(hardwareMap, follower, HardwareController.Team.RED);
    }

    @Override
    public void init_loop() {
        // display telemetry
        displayControls();
        addDefaultTelemetry();

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }

    @Override
    public void loop() {
        // update pathing
        follower.update();
        setTeleOpDrive();

        // update telemetry
        dashboard.sendTelemetryPacket(packet);

        // display telemetry
        displayControls();
        addDefaultTelemetry();

        dashboard.sendTelemetryPacket(packet);
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
        if (robotCentric) {
            // robot-oriented controls will be the same regardless of the side
            follower.manual(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );
        } else {
            // field oriented controls will differ by side and should be overriden
            double[] controls = getDriveControls();
            follower.manual(
                    controls[0],
                    controls[1],
                    controls[2]
            );
        }
    }

    abstract protected void updateControls(); // this should be overriden with non-driving teleop controls (i.e. intake, macros, etc)

    abstract protected void displayControls(); // this should be overriden with driver hub controls

    /**
     * Add and publish default telemetry for teleop
     */
    protected void addDefaultTelemetry() {
        // this method should be modified for game-specific data
        packet.put("Position (In)", follower.pose());
        packet.put("Velocity (In/Sec)", follower.pose());
        packet.put("Using Default Starting Pose", !blackboard.containsKey("Auto Pose"));
    }
}


