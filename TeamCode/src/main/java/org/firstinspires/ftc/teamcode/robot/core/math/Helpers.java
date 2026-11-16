package org.firstinspires.ftc.teamcode.robot.core.math;

public class Helpers {
    private Helpers() {
        throw new AssertionError("No instances");
    }

    public static double wrap(double value, double lower, double upper) {
        // resolve
        // convert to range space, floor, then multiply back out
        double range = upper - lower;
        return value - range * Math.floor((value - lower) / range);
    }

    public static int wrap(int value, int lower, int upper) {
        // resolve
        // convert to range space, floor, then multiply back out
        int range = upper - lower;
        return lower + Math.floorMod(value - lower, range);
    }

    public static long wrap(long value, long lower, long upper) {
        // resolve
        // convert to range space, floor, then multiply back out
        long range = upper - lower;
        return lower + Math.floorMod(value - lower, range);
    }

    public static double clamp(double value, double lower, double upper) {
        return Math.max(lower, Math.min(value, upper));
    }

    public static int clamp(int value, int lower, int upper) {
        return Math.max(lower, Math.min(value, upper));
    }

    public static long clamp(long value, long lower, long upper) {
        return Math.max(lower, Math.min(value, upper));
    }

    public static double snap(double value, double a, double b) {
        double da = Math.abs(value - a), db = Math.abs(value - b);
        return (db < da) ? a : b;
    }

    public static int snap(int value, int a, int b) {
        int da = Math.abs(value - a), db = Math.abs(value - b);
        return (db < da) ? a : b;
    }

    public static long snap(long value, long a, long b) {
        long da = Math.abs(value - a), db = Math.abs(value - b);
        return (db < da) ? a : b;
    }

    public static boolean between(double value, double min, double max) { return min <= value && value <= max; }
    public static boolean between(int value, int min, int max)          { return min <= value && value <= max; }
    public static boolean between(long value, long min, long max)       { return min <= value && value <= max; }
}
