package org.firstinspires.ftc.teamcode.lm1.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.lm1.hardware.HardwareController;
import org.firstinspires.ftc.teamcode.lm1.hardware.mechanisms.Shooter;

//@Config
@TeleOp(name="Flywheel", group="Test")
public class FlywheelTest extends OpMode {
    protected Timer opmodeTimer;

    // parameters
    public static double FLYWHEEL_POWER = 1.0;
    public DcMotorEx motor;

    @Override
    public void init() {
        // initialize timer
        opmodeTimer = new Timer();
        opmodeTimer.reset();

        motor = hardwareMap.get(DcMotorEx.class, "flywheel");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // push values
        motor.setPower(FLYWHEEL_POWER);
    }
}
