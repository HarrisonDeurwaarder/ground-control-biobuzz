package org.firstinspires.ftc.teamcode.robot.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
@TeleOp(name="Motor", group="Test")
public class MotorTester extends OpMode {
    protected Timer opmodeTimer;
    private TelemetryManager panels;

    // parameters
    public static double POWER = 0.0;
    public DcMotorEx motor;

    @Override
    public void init() {
        // initialize timer
        opmodeTimer = new Timer();
        opmodeTimer.reset();

        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        motor = hardwareMap.get(DcMotorEx.class, "motor");
    }

    @Override
    public void loop() {
        // push values
        motor.setPower(POWER);

        panels.debug("Power", POWER);
        panels.debug("Power Read", motor.getPower());
        panels.update();

        //dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Power Read", POWER);
        telemetry.update();
    }
}
