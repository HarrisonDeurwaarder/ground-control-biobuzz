package org.firstinspires.ftc.teamcode.lm1.math;

public class Helpers {
    private Helpers() {
        throw new AssertionError("No instances");
    }

    public static double boundDegrees(double degrees) {
        // resolve
        // if degrees is negative, reduce and invert
        if (degrees < 0) {
            degrees = 360 - (degrees % 360);
            // if degrees is over 360, reduce to 0-360
        } else if (degrees > 360) {
            degrees %= 360;
        }
        return degrees;
    }

    public static double snapDegrees(double degrees, double lower, double upper) {
        return Math.max(lower, Math.min(degrees, upper));
    }

    public static double snapDegreesAxon(double degrees) {
        return snapDegrees(degrees, 0.0, 350.0);
    }
}
