package org.firstinspires.ftc.teamcode.robot.tests;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.robot.teleop.utils.TeleOpBase;

@TeleOp(name="Basic Intake", group="Test")
public class BasicIntake extends TeleOpBase {
    Pose startingPose = new Pose(0.0, 0.0);
    Intake intake;

    @Override
    public void init() {
        defaultStartingPose = startingPose;
        super.init();
        intake = hwc.intake();
    }

    @Override
    public void updateControls() {
        // TRIGGER INTAKE
        if (gamepad1.leftTriggerWasPressed()) { intake.setPower(Intake.PowerState.ON); }
        else if (gamepad1.leftTriggerWasReleased()) { intake.setPower(Intake.PowerState.OFF); }
        // TOGGLE INTAKE/OUTTAKE
        if (gamepad1.leftBumperWasPressed()) { intake.toggleDirection(); }
        // TOGGLE FLOWER
        if (gamepad1.bWasPressed()) { intake.toggleFlower(); }
    }

    @Override
    public void displayControls() {
        telemetry.addLine("    Power - LT");
        telemetry.addLine("Direction - A");
        telemetry.addLine("   Flower - B");
    }
}
