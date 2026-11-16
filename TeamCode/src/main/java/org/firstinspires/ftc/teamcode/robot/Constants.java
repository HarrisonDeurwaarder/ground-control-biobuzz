package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;
import java.util.concurrent.TimeUnit;

@Configurable
public class Constants {
    private Constants() { throw new AssertionError("No instances"); }

    public static class Pedro {
        private Pedro() { throw new AssertionError("No instances"); }

        /* MECANUM */

        public static MecanumConfig drivetrain = new MecanumConfig(
                c -> {
                    c.frontLeftName.set("left_front");
                    c.backLeftName.set("left_back");
                    c.frontRightName.set("right_front");
                    c.backRightName.set("right_back");

                    c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                    c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                    c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                    c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                }
        );

        /* ODOMETRY */
        public static PinpointConfig pinpoint = new PinpointConfig(c -> {
            // required: get() throws IllegalStateException if unset
            c.name.set("pinpoint");
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.xPodOffset.set(1.0);     // passed to Pinpoint setOffsets(x, y): was forwardPodY
            c.yPodOffset.set(-2.5);    // was strafePodX

            // optional, shown at defaults
            c.offsetUnits.set(DistanceUnit.INCH);
            c.globalDistanceUnit.set(DistanceUnit.INCH);           // unit for all poses Pedro reports
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            c.ticksPerUnit.set(OptionalDouble.empty());            // OptionalDouble.of(x) overrides podType
            c.encoderResolutionUnit.set(DistanceUnit.INCH);        // unit for ticksPerUnit
            c.resetMode.set(PinpointLocalizer.ResetMode.RECALIBRATE_IMU);  // or RESET_AND_RECALIBRATE_IMU, NONE
        });

        /* FORESIGHT */

        public static ForesightConfig foresight = new ForesightConfig(c -> {
            // end-of-path criteria (defaults shown)
            c.parametricTConstraint.set(0.025);     // fraction of path left
            c.velocityConstraint.set(0.1);          // units/s
            c.translationalConstraint.set(0.1);     // units
            c.headingConstraint.set(0.007);         // rad
            c.timeoutConstraint.set(100.0);         // ms

            // motion limits (default: ForesightConfig.Constraint.NONE = unlimited)
            c.maxVelocityConstraint.set(ForesightConfig.Constraint.NONE);      // units/s
            c.maxAccelerationConstraint.set(ForesightConfig.Constraint.NONE);  // units/s^2
            c.maxDecelerationConstraint.set(ForesightConfig.Constraint.NONE);   // can't exceed natural deceleration
            c.maxPathSpeed.set(0.8);                // fraction of max speed

            // braking behaviour (defaults)
            c.brakeAtEnd.set(true);
            c.brakeAggression.set(1.0);             // >1 allows overshoot, <1 undershoots
            c.maxBrakingPower.set(0.2);
            c.pathSkip.set(true);                   // carry momentum into the next path

            // required, values come from the tuning OpModes:
            // headingFeedback, forwardTranslational, strafeTranslational, brake, coast (Controllers),
            // linear/quadraticBrakeCoefficients, headingBrakeCoefficients,
            // maxAchievableForward/StrafeVelocity, naturalForward/StrafeDeceleration
        });
    }

    public static class MissionControl {
        private MissionControl() { throw new AssertionError("No instances"); }

        // DT
        public static long MIN_DT = 1_000_000L;
        public static long MAX_DT = 50_000_000L;

        // MANIFEST
        public static String PREFIX = "entry";
    }

    public static class Hardware {
        private Hardware() { throw new AssertionError("No instances"); }

        public static double AXON_MIN_DEG = 0.0;
        public static double AXON_MAX_DEG = 350.0;
    }
}
