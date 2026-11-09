package org.firstinspires.ftc.teamcode.lm1.tests;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lm1.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.lm1.teleop.utils.TeleOpBase;

@TeleOp(name="Basic Intake", group="Test")
public class BasicIntake extends TeleOpBase {
    Pose startingPose = new Pose(0.0, 0.0);
    Intake intake;

    @Override
    public void init() {
        defaultStartingPose = startingPose;
        super.init();
        intake = hwc.getIntake();
    }

    @Override
    public void updateControls() {
        // TRIGGER INTAKE
        if (gamepad1.right_trigger_pressed) { intake.setPower(Intake.PowerState.ON); }
        // TOGGLE INTAKE/OUTTAKE
        if (gamepad1.aWasPressed()) { intake.toggleDirection(); }
    }

    @Override
    public void displayControls() {
        telemetry.addLine("    Power - RT");
        telemetry.addLine("Direction - A");
    }
}
