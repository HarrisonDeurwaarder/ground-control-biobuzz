package org.firstinspires.ftc.teamcode.lm1.teleop;

import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.HOOD_DEFAULT_ANGLE;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.lm1.hardwareutils.HardwareController;
import org.firstinspires.ftc.teamcode.lm1.teleop.utils.TeleOpBase;

@Disabled
public class Red extends TeleOpBase {

    // initalizer block for overriding defaults or declaring initialized variables
    {
        defaultStartingPose = new Pose(0.0, 0.0, 0.0);
        recalibratedPose = new Pose(0.0, 0.0, 0.0);
    }

    @Override
    public void init() {
        super.init();
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
        // add extra loop functionality below, or delete this method signature
        // ...
    }

    @Override
    public void stop() {
        super.stop();

        // disable all mechanisms
        hwc.setIntakeState(HardwareController.IntakeState.OFF);
        hwc.setIntakeState(HardwareController.IntakeState.UP);

        hwc.setFlowerState(HardwareController.FlowerState.UP);

        hwc.setGateState(HardwareController.GateState.CLOSED);

        hwc.setHoodAngle(HOOD_DEFAULT_ANGLE);
        hwc.setFlywheelSpeed(0.0);
    }

    @Override
    protected double[] getDriveControls() {
        // field-oriented is not allowed
        double[] controls = {
                0.0,
                0.0,
                0.0
        };
        return controls;
    }

    protected void updateControls() {
        // add all non-drivetrain controls in here
        // i.e. intake, macros, etc

        // intake on/off
        if (gamepad1.leftTriggerWasPressed()) {
            hwc.setIntakeState(HardwareController.IntakeState.ON);
        } else if (gamepad1.leftTriggerWasReleased()) {
            hwc.setIntakeState(HardwareController.IntakeState.OFF);
        }

        // feed
        if (gamepad1.rightTriggerWasPressed()) {
            hwc.setGateState(HardwareController.GateState.OPEN);
        } else if (gamepad1.rightTriggerWasPressed()) {
            hwc.setGateState(HardwareController.GateState.CLOSED);
        }

        // enable flywheel/hood
        if (gamepad1.aWasPressed()) {
            hwc.flywheelActive = !hwc.flywheelActive;
            hwc.hoodActive = !hwc.hoodActive;
        }

        // enable rotation
        if (gamepad1.bWasPressed()) {
            hwc.rotActive = !hwc.rotActive;
        }

        // recalibrate
        if (gamepad1.left_bumper && gamepad1.right_bumper) follower.setPose(recalibratedPose); // tell pedro that it is at the known recalibration pose
    }

    protected void displayControls() {
        // fill with control information to be displayed on the driver hub
        telemetry.addLine("Toggle Flywheel      - A");
        telemetry.addLine("Toggle Turret        - B");
        telemetry.addLine("Recalibrate Odometry - LB & RB");
        // if multiple drivers are used, consider adding a section header to differentiate
        // ...
    }
}


