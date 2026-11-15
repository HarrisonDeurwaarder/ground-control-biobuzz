package org.firstinspires.ftc.teamcode.robot.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.robot.hardware.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.robot.teleop.utils.TeleOpBase;

@TeleOp(name="Red", group="Competition")
public class Red extends TeleOpBase {
    Shooter shooter;
    Intake intake;
    Pose startingPose = new Pose(0.0, 0.0, 0.0);

    @Override
    protected Pose getStartingPose() {
        return startingPose;
    }

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
        super.loop();double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = -gamepad1.left_stick_x;
        double yaw     =  -gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        // Send calculated power to wheels
        hwc.leftFront.setPower(frontLeftPower);
        hwc.rightFront.setPower(frontRightPower);
        hwc.leftBack.setPower(backLeftPower);
        hwc.rightBack.setPower(backRightPower);
        // run periodic functionality
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
        if (gamepad1.left_trigger_pressed) {
            intake.setDirection(Intake.DirectionState.INTAKE);
            intake.setPower(Intake.PowerState.ON);
        }

        // power (outtake)
        else if (gamepad1.left_bumper) {
            intake.setDirection(Intake.DirectionState.OUTTAKE);
            intake.setPower(Intake.PowerState.ON);
        }
        else { intake.setPower(Intake.PowerState.OFF); }

        /* SHOOTER */

        // feed
        if (gamepad1.right_trigger_pressed) { shooter.setFeeding(Shooter.FeedingState.FEED); }
        else { shooter.setFeeding(Shooter.FeedingState.HOLD); }

        // toggle turret
        //if (gamepad1.xWasPressed()) { shooter.toggleTurretTracking(); }

        // toggle flywheel
        //if (gamepad1.yWasPressed()) { shooter.toggleFlywheelTracking(); }
    }

    protected void displayControls() {
        // fill with control information to be displayed on the driver hub. example:
        telemetry.addLine("Recalibrate Odometry - LB & RB");
        telemetry.addData("Gate State", shooter.getFeeding());
        // if multiple drivers are used, consider adding a section header to differentiate
        // ...
    }
}


