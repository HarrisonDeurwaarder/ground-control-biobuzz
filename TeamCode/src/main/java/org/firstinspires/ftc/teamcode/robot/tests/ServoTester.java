package org.firstinspires.ftc.teamcode.robot.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Configurable
@TeleOp(name="Servo", group="Test")
public class ServoTester extends OpMode {
    protected Timer opmodeTimer;
    private TelemetryManager panels;

    // parameters
    public static double POSITION = 0.0;
    public ServoImplEx servo;

    @Override
    public void init() {
        // initialize timer
        opmodeTimer = new Timer();
        opmodeTimer.reset();

        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        servo = hardwareMap.get(ServoImplEx.class, "servo");
    }

    @Override
    public void loop() {
        // push values
        servo.setPosition(POSITION);

        panels.debug("Position", POSITION);
        panels.debug("Position Read", servo.getPosition());
        panels.update();

        //dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Position", servo.getPosition());
        telemetry.addData("Position Read", POSITION);
        telemetry.update();
    }
}
