package org.firstinspires.ftc.teamcode.robot.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Configurable
@TeleOp(name="Servo (Unpowered)", group="Test")
public class UnpoweredServoTester extends OpMode {
    protected Timer opmodeTimer;
    private TelemetryManager panels;

    // parameters
    public ServoImplEx servo;

    @Override
    public void init() {
        // initialize timer
        opmodeTimer = new Timer();
        opmodeTimer.reset();

        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        servo = hardwareMap.get(ServoImplEx.class, "servo");
        servo.setPwmDisable();
    }

    @Override
    public void loop() {
        panels.debug("Position Read", servo.getPosition() * 350.0);
        panels.update();

        //dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Position", servo.getPosition() * 350.0);
        telemetry.update();
    }
}
