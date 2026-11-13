package org.firstinspires.ftc.teamcode.lm1.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.lm1.hardware.HardwareController;
import org.firstinspires.ftc.teamcode.lm1.hardware.mechanisms.Shooter;

@Configurable
@TeleOp(name="Servo", group="Test")
public class ServoTester extends OpMode {
    protected Timer opmodeTimer;
    //protected TelemetryPacket packet;
    //protected FtcDashboard dashboard;

    // parameters
    public static double POSITION = 0.0;
    public ServoImplEx servo;
    private int i = 0;

    @Override
    public void init() {
        // initialize timer
        opmodeTimer = new Timer();
        opmodeTimer.reset();

        //packet = new TelemetryPacket();
        //dashboard = FtcDashboard.getInstance();

        servo = hardwareMap.get(ServoImplEx.class, "servo");
    }

    @Override
    public void loop() {
        // push values
        servo.setPosition(POSITION);
        i++;

        //packet = new TelemetryPacket();
        //packet.put("Position", servo.getPosition());
        //packet.put("Position2", POSITION);
        //packet.put("i", i);

        //dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Position", servo.getPosition());
        telemetry.addData("Position2", POSITION);
        telemetry.addData("i", i);
        telemetry.update();
    }
}
