package org.firstinspires.ftc.teamcode.lm1.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lm1.hardware.HardwareController;
import org.firstinspires.ftc.teamcode.lm1.hardware.mechanisms.Shooter;

@Configurable
@TeleOp(name="Turret Functions", group="Test")
public class TurretFunctionTest extends OpMode {
    protected HardwareController hwc;
    protected Follower follower;
    protected Shooter shooter;

    // teleop timer
    protected Timer opmodeTimer;

    // telemetry objects
    //protected TelemetryPacket packet;
    //protected FtcDashboard dashboard;

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
        //packet = new TelemetryPacket();
        //dashboard = FtcDashboard.getInstance();

        // initialize hwc
        hwc = new HardwareController(hardwareMap, follower, HardwareController.Team.RED);
        shooter = hwc.shooter();

    }

    @Override
    public void loop() {
        // push values
        shooter.setTurretAngle(TURRET_ANGLE);
        shooter.setFlywheelCommand(FLYWHEEL_SPEED);
        // update flywheel PID
        shooter.updateFlywheelEffort();
        // update telemetry
        updateTelemetry();
        //dashboard.sendTelemetryPacket(packet);
    }

    private void updateTelemetry() {
        //packet.put("Turret Angle   (DEG)", shooter.getTurretAngle());
        //packet.put("Flywheel Speed (RPM)", shooter.getFlywheelVelocity());
    }
}
