package org.firstinspires.ftc.teamcode.lm1.hardware.mechanisms;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lm1.hardware.control.ShootingWhileMoving;
import org.firstinspires.ftc.teamcode.lm1.math.transforms.Compose;
import org.firstinspires.ftc.teamcode.lm1.math.transforms.Transform;
import org.firstinspires.ftc.teamcode.lm1.models.LinearInterpolation;
import org.firstinspires.ftc.teamcode.lm1.math.Helpers;
import org.firstinspires.ftc.teamcode.lm1.hardware.control.pid.PIDController;
import org.firstinspires.ftc.teamcode.lm1.hardware.control.pid.PIDGains;
import static org.firstinspires.ftc.teamcode.lm1.math.linalg.Point.point;


public class Shooter {

    // transfer
    public static double GATE_FEEDING_DEGREES = 0.0, GATE_HOLDING_DEGREES = 0.0;

    // flywheel
    public static double FLYWHEEL_GEARING = 1.0;
    public static PIDGains flywheelGains = new PIDGains(0.0, 0.0, 0.0, 0.0, 0.0);
    private final LinearInterpolation flywheelInterpolation = new LinearInterpolation(
            point(0.0, 0.0),
            point(1.0, 0.0)
    );

    // turret
    public static double TURRET_OFFSET1 = 0.0, TURRET_OFFSET2 = 0.0, TURRET_GEARING = 0.0;
    public static ShootingWhileMoving.SWMParameters swmParams = new ShootingWhileMoving.SWMParameters(0.0, 0.0);
    private final ShootingWhileMoving swm = new ShootingWhileMoving(swmParams);

    // hood
    public static double HOOD_OFFSET = 0.0, HOOD_GEARING = 1.0;
    private final LinearInterpolation hoodInterpolation = new LinearInterpolation(
            point(0.0, 0.0),
            point(1.0, 0.0)
    );

    public enum FeedingState { FEED, HOLD }
    public enum TrackingState { TRACK, STATIC, OFF }

    private final DcMotorEx flywheel;
    private final ServoImplEx gate, turret1, turret2;
    private final PIDController flywheelPID;

    private FeedingState feedingState;
    private TrackingState flywheelState, rotationState, hoodState;

    public Shooter(HardwareMap hardwareMap) {
        // map actuators used by the turret
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        gate =    hardwareMap.get(ServoImplEx.class, "gate");
        turret1 = hardwareMap.get(ServoImplEx.class, "turret1");
        turret2 = hardwareMap.get(ServoImplEx.class, "turret2");
        // create flywheel pid
        flywheelPID = new PIDController(flywheelGains);
    }

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

    public void update() {
        // TRANSFORM GOAL POSE

        // TRANSFORM GOAL POSE FOR FLYWHEEL

        // COMPUTE TURRET ANGLE
        if (rotationState == TrackingState.TRACK) {}
        // APPLY FLYWHEEL REGRESSION + INTERP

        // APPLY HOOD INTERP
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

    public double computeTurretAngle(Vector turret, Vector goal) { return turret.angleTo(goal); }

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
