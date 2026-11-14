package org.firstinspires.ftc.teamcode.robot.teleop;

import org.firstinspires.ftc.teamcode.robot.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.robot.hardware.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.robot.teleop.utils.TeleOpBase;

class Red extends TeleOpBase {
    Shooter shooter;
    Intake intake;

    @Override
    public void init() {
        super.init();
        // store references for mechanisms
        shooter = hwc.shooter();
        intake = hwc.intake();
        // add extra init functionality below, or delete this method signature
        // ...
    }

    @Override
    public void init_loop() {
        super.init_loop();
        // add extra init_loop functionality below, or delete this method signature
        // ...
    }

    @Override
    public void start() {
        // TeleOpBase doesn't implement start(), so no super method must be called
        // add start functionality below, or delete this method signature
        // ...
    }

    @Override
    public void loop() {
        super.loop();
        // run periodic functionality
        shooter.command();
        shooter.update();
        // add extra loop functionality below, or delete this method signature
        // ...
    }

    @Override
    public void stop() {
        super.stop();
        hwc.disable();
        // add extra stop functionality below, or delete this method signature
        // ...
    }

    protected void updateControls() {
        // add all non-drivetrain controls in here
        // i.e. intake, macros, etc

        /* INTAKE */

        // power (intake)
        if (gamepad1.leftTriggerWasPressed()) {
            intake.setDirection(Intake.DirectionState.INTAKE);
            intake.setPower(Intake.PowerState.ON);
        }

        // power (outtake)
        else if (gamepad1.leftBumperWasPressed()) {
            intake.setDirection(Intake.DirectionState.OUTTAKE);
            intake.setPower(Intake.PowerState.ON);
        }
        else { intake.setPower(Intake.PowerState.OFF); }

        /* SHOOTER */

        // feed
        if (gamepad1.left_trigger_pressed) { shooter.setFeeding(Shooter.FeedingState.FEED); }
        else { shooter.setFeeding(Shooter.FeedingState.HOLD); }

        // toggle turret
        if (gamepad1.xWasPressed()) { shooter.toggleTurretTracking(); }

        // toggle flywheel
        if (gamepad1.yWasPressed()) { shooter.toggleFlywheelTracking(); }
    }

    protected void displayControls() {
        // fill with control information to be displayed on the driver hub. example:
        telemetry.addLine("Recalibrate Odometry - LB & RB");
        telemetry.addLine("Recalibrate Odometry - LB & RB");
        // if multiple drivers are used, consider adding a section header to differentiate
        // ...
    }
}


