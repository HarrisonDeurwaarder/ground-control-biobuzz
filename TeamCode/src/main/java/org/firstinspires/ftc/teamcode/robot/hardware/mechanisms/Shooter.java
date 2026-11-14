package org.firstinspires.ftc.teamcode.robot.hardware.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.hardware.control.ShootingWhileMoving;
import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;
import org.firstinspires.ftc.teamcode.robot.models.LinearInterpolation;
import org.firstinspires.ftc.teamcode.robot.math.Helpers;
import org.firstinspires.ftc.teamcode.robot.hardware.control.pid.PIDController;
import org.firstinspires.ftc.teamcode.robot.hardware.control.pid.PIDGains;
import static org.firstinspires.ftc.teamcode.robot.math.linalg.Point.point;


public class Shooter {

    // transfer
    public static double GATE_FEEDING_DEGREES = 0.0, GATE_HOLDING_DEGREES = 0.0;

    // flywheel
    public static double FLYWHEEL_GEARING = 1.0;
    public static PIDGains flywheelGains = new PIDGains(0.0, 0.0, 0.0, 0.0, 0.0);
    public static double toF = 1.0;
    private final LinearInterpolation flywheelInterpolation = new LinearInterpolation(
            point(0.0, 0.0),
            point(1.0, 0.0)
    );

    // turret
    public static double TURRET_OFFSET1 = 0.0, TURRET_OFFSET2 = 0.0, TURRET_GEARING = 0.0;
    public static ShootingWhileMoving.SWMParameters swmParams = new ShootingWhileMoving.SWMParameters(0.0, 0.0);
    private final ShootingWhileMoving swm = new ShootingWhileMoving(swmParams);

    public enum FeedingState { FEED, HOLD }
    public enum TrackingState { TRACK, STATIC, OFF }

    private final DcMotorEx flywheel;
    private final ServoImplEx gate, turret1, turret2;
    private final PIDController flywheelPID;
    private final Follower follower;

    private FeedingState feedingState;
    private TrackingState flywheelState, rotationState;

    // boolean flags
    public boolean useSwM = false;

    // scratch variables
    private Vector poseVect = Vector.zero(3);

    public Shooter(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        // map actuators used by the turret
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        gate =    hardwareMap.get(ServoImplEx.class, "gate");
        turret1 = hardwareMap.get(ServoImplEx.class, "turret1");
        turret2 = hardwareMap.get(ServoImplEx.class, "turret2");
        // create flywheel pid
        flywheelPID = new PIDController(flywheelGains);
    }

    /* SETTERS */

    public void setDefaults() {
        // FLYWHEEL
        // reset encoder value
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // disable automatic velocity control
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // set correct direction for shooting
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        setFlywheelTracking(TrackingState.TRACK);

        // GATE
        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axons use a different PWM range than GoBildas
        setFeeding(FeedingState.HOLD);

        // TURRET
        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        setTurretTracking(TrackingState.TRACK);
    }

    public void disable() {
        // enable braking for a quick stop
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // setting to zero will brake now
        flywheel.setPower(0.0);

        // snap turret to default
        setTurretAngle(0.0);
    }

    /* FUNCTIONAL METHODS */

    public void command(Vector goal) {
        // COMPUTE SWM ROBOT POSE
        Pose pose = follower.pose();
        if (useSwM) { pose = swm.applyToPose(pose, Vector.zero(3), follower.velocity(), toF); }

        // COMPUTE TURRET ANGLE
        poseVect.set(pose.x(), pose.y());
        double angle = computeTurretDegrees(poseVect, goal);
        setTurretAngle(angle);

        // COMPUTE FLYWHEEL VELOCITY
        double rpm = flywheelInterpolation.infer(poseVect.distance(goal));
        setFlywheelCommand(rpm);
    }

    public void update() {
        updateFlywheelEffort();
    }

    /* FEEDING */

    public FeedingState getFeeding() { return feedingState; }
    public void setFeeding(FeedingState state) {
        feedingState = state;
        // apply
        if (state == FeedingState.FEED) {
            gate.setPosition(GATE_FEEDING_DEGREES);
        } else if (state == FeedingState.HOLD) {
            gate.setPosition(GATE_HOLDING_DEGREES);
        }
    }
    public void toggleFeeding() {
        setFeeding((getFeeding() == FeedingState.FEED) ? FeedingState.HOLD : FeedingState.FEED);
    }

    /* FLYWHEEL */

    public TrackingState getFlywheelTracking() { return flywheelState; }
    public void setFlywheelTracking(TrackingState state) { flywheelState = state; }
    public void toggleFlywheelTracking() {
        setFlywheelTracking((getFlywheelTracking() == TrackingState.TRACK) ? TrackingState.STATIC : TrackingState.TRACK);
    }

    /* TURRET */

    public TrackingState getTurretTracking() { return rotationState; }
    public void setTurretTracking(TrackingState state) { rotationState = state; }
    public void toggleTurretTracking() {
        setTurretTracking((getTurretTracking() == TrackingState.TRACK) ? TrackingState.STATIC : TrackingState.TRACK);
    }

    /* ALGORITHMS */

    public void interpolateFlywheel(double distance) {
        // compute rpm from interpolation
        double rpm = flywheelInterpolation.infer(distance);
        // send cmd
        setFlywheelCommand(rpm);
    }

    public double computeTurretDegrees(Vector turret, Vector goal) { return Math.toDegrees(turret.angle(goal)); }

    /* HELPERS */

    public double getFlywheelVelocity() {
        // convert degrees/second to rpm
        return (flywheel.getVelocity(AngleUnit.DEGREES) * FLYWHEEL_GEARING) / 6.0; // dps * (1rot/360deg) * (60sec/min)
    }

    public void setFlywheelCommand(double rpm) { flywheelPID.changeSetpoint(rpm); }

    public void updateFlywheelEffort() {
        double power = flywheelPID.compute(
                getFlywheelVelocity()
        );
        flywheel.setPower(power);
    }

    public double getTurretAngle() {
        double position = (turret1.getPosition() + turret2.getPosition()) / 2.0;
        // negate known offset
        return position * 350.0 * TURRET_GEARING + (TURRET_OFFSET1 + TURRET_OFFSET2) / 2.0;
    }

    public void setTurretAngle(double degrees) {
        // apply safety transforms
        double degrees1 = Helpers.snapDegreesAxon(
                Helpers.boundDegrees(degrees - TURRET_OFFSET1)
        );
        double degrees2 = Helpers.snapDegreesAxon(
                Helpers.boundDegrees(degrees - TURRET_OFFSET2)
        );
        // push to Axons and apply known offsets
        turret1.setPosition(degrees1 / 350.0);
        turret2.setPosition(degrees2 / 350.0);
    }
}
