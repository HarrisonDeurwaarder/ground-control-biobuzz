package org.firstinspires.ftc.teamcode.robot.tests;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.robot.hardware.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.robot.teleop.utils.TeleOpBase;

@TeleOp(name="Basic Intake & Turret", group="Test")
public class BasicIntakeAndTurret extends TeleOpBase {
    Pose startingPose = new Pose(0.0, 0.0);
    Intake intake;
    Shooter shooter;

    @Override
    public void init() {
        defaultStartingPose = startingPose;
        super.init();
        intake = hwc.intake();
        shooter = hwc.shooter();
    }

    @Override
    public void loop() {
        shooter.update();
        super.loop();
    }

    @Override
    public void updateControls() {
        // TRIGGER INTAKE
        if (gamepad1.leftTriggerWasPressed()) { intake.setPower(Intake.PowerState.ON); }
        else if (gamepad1.leftTriggerWasReleased()) { intake.setPower(Intake.PowerState.OFF); }
        // TOGGLE INTAKE/OUTTAKE
        if (gamepad1.leftBumperWasPressed()) { intake.toggleDirection(); }

        // SHOOT
        if (gamepad1.right_trigger > 0.1) { shooter.setFeeding(Shooter.FeedingState.FEED); }
        else { shooter.setFeeding(Shooter.FeedingState.HOLD); }

        // TRACKING/STATIC
        if (gamepad1.aWasPressed()) { shooter.toggleFlywheelTracking(); }
        if (gamepad1.bWasPressed()) { shooter.toggleTurretTracking(); }

        // OFF
        if (gamepad1.xWasPressed()) { shooter.setFlywheelTracking(Shooter.TrackingState.OFF); }
        if (gamepad1.yWasPressed()) { shooter.setTurretTracking(Shooter.TrackingState.OFF); }
    }

    @Override
    public void displayControls() {
        telemetry.addLine("          Intake - LT");
        telemetry.addLine("Intake Direction - LB");
        telemetry.addLine("            Feed - RT");
        telemetry.addLine("   Turret Toggle - A");
        telemetry.addLine(" Flywheel Toggle - B");
        telemetry.addLine("      Turret Off - X");
        telemetry.addLine("    Flywheel Off - Y");
    }
}
