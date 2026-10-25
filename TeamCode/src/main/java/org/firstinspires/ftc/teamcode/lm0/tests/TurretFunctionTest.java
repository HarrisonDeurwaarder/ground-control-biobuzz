package org.firstinspires.ftc.teamcode.lm0.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareController;

public class TurretFunctionTest extends OpMode {
    protected HardwareController hwc;
    protected Follower follower;

    // teleop timer
    protected Timer opmodeTimer;

    // telemetry objects
    protected TelemetryPacket packet;
    protected FtcDashboard dashboard;

    // parameters
    public static double TURRET_ANGLE = 0.0;
    public static double HOOD_ANGLE = 0.0;
    public static double FLYWHEEL_SPEED = 0.0;

    @Override
    public void init() {
        // initialize timer
        opmodeTimer = new Timer();
        opmodeTimer.reset();
        // initialize telemetry
        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        // initialize hwc
        hwc = new HardwareController(hardwareMap, follower, HardwareController.Team.RED);
    }

    @Override
    public void loop() {
        // push values
        hwc.setHoodAngle(HOOD_ANGLE);
        hwc.setFlywheelSpeed(FLYWHEEL_SPEED);
        // update telemetry
        updateTelemetry();
        dashboard.sendTelemetryPacket(packet);
    }

    private void updateTelemetry() {
        packet.put("Hood Angle     (DEG)", hwc.getHoodAngle());
        packet.put("Flywheel Speed (RPM)", hwc.getFlywheelSpeed());
    }
}
