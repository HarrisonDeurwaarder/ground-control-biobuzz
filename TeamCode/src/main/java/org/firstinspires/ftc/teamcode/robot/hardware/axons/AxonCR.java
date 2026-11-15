package org.firstinspires.ftc.teamcode.robot.hardware.axons;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.robot.hardware.control.pid.PIDController;
import org.firstinspires.ftc.teamcode.robot.hardware.control.pid.PIDGains;

public class AxonCR {
    private CRServoImplEx servo;
    private AnalogInput encoder;
    private double maxEncoderVoltage;

    private PIDController posPID, velPID;

    // trackers
    private

    private AxonCR(CRServoImplEx servo, AnalogInput encoder) {
        this.servo = servo; this.encoder = encoder;
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo.setPwmEnable();
        maxEncoderVoltage = encoder.getMaxVoltage();
    }

    /* GETTERS & SETTERS */

    // position pid
    public void positionGains(PIDGains gains) { posPID = new PIDController(gains); }
    public PIDController positionPID() { return posPID; }

    // velocity pid
    public void velocityGains(PIDGains gains) { velPID = new PIDController(gains); }
    public PIDController velocityPID() { return velPID; }

    public CRServoImplEx servo() { return servo; }

    public double position() { return encoder.getVoltage() / maxEncoderVoltage * 360.0; }
    public void position(double degrees) { posPID.changeSetpoint(degrees); }

    public double power() { return servo.getPower(); }
    public void power(double power) { servo.setPower(power); }

    /* FUNCTIONALS */

    private void update() {

    }
}
