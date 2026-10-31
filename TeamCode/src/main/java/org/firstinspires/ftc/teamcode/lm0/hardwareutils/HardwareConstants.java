package org.firstinspires.ftc.teamcode.lm0.hardwareutils;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.lm0.hardwareutils.control.LinearInterpolation;
import org.firstinspires.ftc.teamcode.lm0.hardwareutils.control.PIDController;

public class HardwareConstants {
    // universal constants
    public static double GOBILDA_TICKS_PER_REVOLUTION = 28.0;

    // mechanical constants
    public static double TURRET_ROT_RATIO = 1.0;
    public static double TURRET_HOOD_RATIO = 0.2;
    public static double TURRET_HOOD_START_DEG = 30;
    public static double FLOWER_MECH_UP = 0.0;
    public static double FLOWER_MECH_DOWN = 0.0;
    public static double INTAKE_UP = 0.0;
    public static double INTAKE_DOWN = 0.0;
    public static double GATE_OPEN = 0.0;
    public static double GATE_CLOSED = 0.0;

    // pose constants
    public static Pose TURRET_OFFSET = new Pose(0.0, 0.0);
    public static Pose BLUE_POSITIVE_GOAL = new Pose(0.0, 0.0);
    public static Pose RED_POSITIVE_GOAL = new Pose(0.0, 0.0);
    public static Pose BLUE_NEGATIVE_GOAL = new Pose(0.0, 0.0);
    public static Pose RED_NEGATIVE_GOAL = new Pose(0.0, 0.0);

    // power constants
    public static double INTAKE_POWER = 0.8;

    // thresholds
    public static double SHOOTING_ZONE_SLOPE = 1.2;
    public static double GOAL_UPDATE_THRESHOLD = 0.0;

    // turret configuration
    public static double FLYWHEEL_DEFAULT_RPM = 40.0;
    public static double HOOD_DEFAULT_ANGLE = 0.0;
    public static double ROT_DEFAULT_ANGLE = 0.0;
    public static PIDController flywheelPID = new PIDController(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static LinearInterpolation flywheelInterp;

    static {
        LinearInterpolation interp = new LinearInterpolation();
        // add points
        interp.addPoint(0.0, 0.0);
        flywheelInterp = interp;
    }
    public static LinearInterpolation hoodInterp;

    static {
        LinearInterpolation interp = new LinearInterpolation();
        // add points
        interp.addPoint(0.0, 0.0);
        hoodInterp = interp;
    }

    // Shooting-while-moving
    public static boolean useTangentialCorrection = false;
    public static double VELOCITY_LATENCY = 0.4;
    public static double ACCELERATION_LATENCY = 0.0;
    public static boolean useRotationalCorrection = false;
    public static double OMEGA_LATENCY = 0.4;
    public static double accelerationTimeCoefficient = 0.0;
}
