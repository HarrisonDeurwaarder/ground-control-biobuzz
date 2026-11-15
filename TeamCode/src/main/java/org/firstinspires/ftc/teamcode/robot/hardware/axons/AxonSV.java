package org.firstinspires.ftc.teamcode.robot.hardware.axons;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.math.Helpers;

public class AxonSV {
    private ServoImplEx servo;
    private AnalogInput encoder;
    private double maxEncoderVoltage;

    public enum Bounder { CLAMP, WRAP };

    public AxonSV(ServoImplEx servo, AnalogInput encoder) {
        this.servo = servo; this.encoder = encoder;
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo.setPwmEnable();
        maxEncoderVoltage = encoder.getMaxVoltage();
    }

    public double getPosition() { return encoder.getVoltage() / maxEncoderVoltage * 360.0; }
    public void setPosition(double degrees, Bounder bounder) {
        // bound degrees
        if (bounder == Bounder.CLAMP) { degrees = Helpers.snapDegreesAxon(degrees); }
        else { degrees = Helpers.boundDegrees(degrees); }
        servo.setPosition(degrees / 350.0);
    }
}
