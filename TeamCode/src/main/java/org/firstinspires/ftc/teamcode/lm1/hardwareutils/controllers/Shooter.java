package org.firstinspires.ftc.teamcode.lm1.hardwareutils.controllers;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lm0.hardwareutils.control.LinearInterpolation;
import org.firstinspires.ftc.teamcode.lm1.hardwareutils.Helpers;
import org.firstinspires.ftc.teamcode.lm1.hardwareutils.control.pid.PIDController;
import org.firstinspires.ftc.teamcode.lm1.hardwareutils.control.pid.PIDGains;

public class Shooter {

    public static double TRANSFER_POWER = 0.8;
    public static double FLYWHEEL_GEARING = 1.0;
    public static PIDGains flywheelGains = new PIDGains(0.0, 0.0, 0.0, 0.0, 0.0);
    public static LinearInterpolation flywheelInterpolation = new LinearInterpolation();
    public static double TURRET_OFFSET1 = 0.0;
    public static double TURRET_OFFSET2 = 0.0;
    public static double TURRET_GEARING = 1.0;
    public static double HOOD_OFFSET = 0.0;
    public static double HOOD_GEARING = 1.0;
    public static LinearInterpolation hoodInterpolation = new LinearInterpolation();

    public enum FeedingState { FEED, HOLD }
    public enum TrackingState { TRACK, STATIC, OFF }

    private DcMotorEx transfer, flywheel1, flywheel2;
    private ServoImplEx rotation1, rotation2, hood;
    private PIDController flywheelPID;

    private FeedingState feedingState;
    private TrackingState flywheelState, rotationState, hoodState;

    public Shooter(DcMotorEx transfer, DcMotorEx flywheel1, DcMotorEx flywheel2, ServoImplEx rotation1, ServoImplEx rotation2, ServoImplEx hood) {
        // map actuators used by the turret
        this.transfer = transfer; this.flywheel1 = flywheel1; this.flywheel2 = flywheel2;
        this.rotation1 = rotation1; this.rotation2 = rotation2; this.hood = hood;
        // create flywheel pid
        flywheelPID = new PIDController(flywheelGains);
    }

    public void setDefaults() {

        setFeeding(FeedingState.HOLD);

        // FLYWHEEL
        // reset encoder value
        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // disable automatic velocity control
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // set correct direction for shooting
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        setFlywheelTracking(TrackingState.TRACK);

        // TURRET
        rotation1.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axons use a different PWM range than GoBildas
        rotation2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        setTurretTracking(TrackingState.TRACK);

        // HOOD
        hood.setPwmRange(new PwmControl.PwmRange(500, 2500));
        setHoodTracking(TrackingState.TRACK);
    }

    public void update() {
        // TRANSFORM GOAL POSE

        // TRANSFORM GOAL POSE FOR FLYWHEEL

        // COMPUTE TURRET ANGLE

        // APPLY FLYWHEEL REGRESSION + INTERP

        // APPLY HOOD INTERP
    }

    /* FEEDING */

    public FeedingState getFeeding() { return feedingState; }
    public void setFeeding(FeedingState state) {
        feedingState = state;
        // apply
        if (state == FeedingState.FEED) {
            transfer.setPower(TRANSFER_POWER);
        } else if (state == FeedingState.HOLD) {
            transfer.setPower(0.0);
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

    /* HOOD */

    public TrackingState getHoodTracking() { return hoodState; }
    public void setHoodTracking(TrackingState state) { hoodState = state; }
    public void toggleHoodTracking() {
        setHoodTracking((getHoodTracking() == TrackingState.TRACK) ? TrackingState.STATIC : TrackingState.TRACK);
    }

    /* ALGORITHMS */

    public void interpolateFlywheel(double distance) {
        // compute rpm from interpolation
        double rpm = flywheelInterpolation.infer(distance);
        // send cmd
        setFlywheelCommand(rpm);
    }

    public double computeTurretAngle(Vector turret, Vector goal) { return turret.angleTo(goal); }

    public void interpolateHood(double distance) {
        // compute degrees from interpolation
        double degrees = hoodInterpolation.infer(distance);
        // send cmd
        setHoodAngle(degrees);
    }

    /*
    * INTERNAL METHODS
    */

    private double getFlywheelVelocity() {
        double avg = (flywheel1.getVelocity(AngleUnit.DEGREES) + flywheel2.getVelocity(AngleUnit.DEGREES)) / 2.0;
        // convert degrees/second to rpm
        return (avg * FLYWHEEL_GEARING) / 6.0; // dps * (1rot/360deg) * (60sec/min)
    }

    private void setFlywheelCommand(double rpm) { flywheelPID.changeSetpoint(rpm); }

    private void updateFlywheelEffort() {
        double power = flywheelPID.compute(
                getFlywheelVelocity()
        );
        flywheel1.setPower(power);
        flywheel2.setPower(power);
    }

    private double getTurretAngle() {
        double position = (rotation1.getPosition() + rotation2.getPosition()) / 2.0;
        // negate known offset
        return position * 350.0 * TURRET_GEARING + (TURRET_OFFSET1 + TURRET_OFFSET2) / 2.0;
    }

    private void setTurretAngle(double degrees) {
        // apply safety transforms
        double degrees1 = Helpers.snapDegreesAxon(degrees - TURRET_OFFSET1);
        double degrees2 = Helpers.snapDegreesAxon(degrees - TURRET_OFFSET2);
        // push to Axons and apply known offsets
        rotation1.setPosition(degrees1 / 350.0);
        rotation2.setPosition(degrees2 / 350.0);
    }

    private double getHoodAngle() {
        double position = hood.getPosition();
        // transform
        return position * HOOD_GEARING - HOOD_OFFSET;
    }

    private void setHoodAngle(double degrees) {
        // apply safety transforms
        degrees = Helpers.snapDegreesAxon(degrees - HOOD_OFFSET);
        // push to Axons and apply known offsets
        hood.setPosition(degrees / 350.0);
    }
}
