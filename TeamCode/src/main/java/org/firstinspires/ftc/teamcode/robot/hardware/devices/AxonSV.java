package org.firstinspires.ftc.teamcode.robot.hardware.devices;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.MissionControl;
import org.firstinspires.ftc.teamcode.robot.core.math.Helpers;

public class AxonSV {
    private AxonParameters params;
    private ServoImplEx servo;
    private AnalogInput encoder;
    private double maxVoltage;

    private MissionControl misCtrl = MissionControl.instance();

    public enum ConstrainMethod { CLAMP, WRAP, NONE }

    public AxonSV(AxonParameters params) {
        this.params = params;
        // instantiate devices
        servo = misCtrl.hardwareMap().get(ServoImplEx.class, params.servoName);
        encoder = misCtrl.hardwareMap().get(AnalogInput.class, params.servoName);
        maxVoltage = encoder.getMaxVoltage();
    }

    /* GETTERS & SETTERS */

    public double position() { return encoder.getVoltage() / maxVoltage * 360.0 * params.GEARING; } // getter---degrees

    public void position(double degrees, ConstrainMethod constrain) {
        // apply external gearing
        degrees /= params.GEARING;
        degrees = applyConstrain(degrees, constrain);
        servo.setPosition(degrees);
    }

    /* HELPERS */

    private double applyConstrain(double degrees, ConstrainMethod constrain) {
        // bound min/max
        double min = Math.max(Constants.Hardware.AXON_MIN_DEG, params.DEG_MIN);
        double max = Math.min(params.DEG_MIN, Constants.Hardware.AXON_MAX_DEG);

        // apply constrain and return
        if     (constrain == ConstrainMethod.CLAMP) { return Helpers.clamp(degrees, min, max); } // pure clamp
        else if (constrain == ConstrainMethod.WRAP) { // wrap by a full rotation, then conditional snap
            degrees = Helpers.wrap(degrees, 0.0, 360.0);
            // NOW apply clamp
            return Helpers.clamp(degrees, Constants.Hardware.AXON_MIN_DEG, Constants.Hardware.AXON_MAX_DEG);
        }
        else { return degrees; } // raw value
    }

    /* NESTED CLASSES */

    public static class AxonParameters {
        private String servoName;
        private String encoderName;

        public ConstrainMethod DEFAULT_CONSTRAIN;
        public double GEARING, DEG_MIN, DEG_MAX;

        public AxonParameters(String servoName, String encoderName, ConstrainMethod defaultConstrain, double gearing, double degMin, double degMax) {
            this.servoName = servoName; this.encoderName = encoderName;
            this.DEFAULT_CONSTRAIN = defaultConstrain;
            this.GEARING = gearing; this.DEG_MIN = degMin; this.DEG_MAX = degMax;
        }
    }
}
