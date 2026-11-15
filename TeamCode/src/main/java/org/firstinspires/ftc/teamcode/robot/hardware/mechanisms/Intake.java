package org.firstinspires.ftc.teamcode.robot.hardware.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.math.Helpers;

public class Intake {

    // flower
    public static double FLOWER_UP_DEGREES = 0.0;
    public static double FLOWER_DOWN_DEGREES = 0.0;
    public static double FLOWER_GEARING = 1.0;

    // defaults
    public static double INTAKE_POWER = 0.8;
    public static DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction OUTTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public enum     PowerState { ON, OFF }
    public enum DirectionState { INTAKE, OUTTAKE }

    private PowerState         powerState = PowerState.OFF;
    private DirectionState directionState = DirectionState.INTAKE;

    DcMotorEx intake1, intake2;

    public Intake(HardwareMap hardwareMap) {
        // map actuators used by the intake
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
    }

    /* SETTERS */

    public void setDefaults() {
        // INTAKE MOTOR
        setDirection(DirectionState.INTAKE);
        setPower(PowerState.OFF);
    }

    public void disable() {
        intake1.setPower(0.0);
        intake2.setPower(0.0);
    }

    /* POWER */
    public PowerState getPower() { return powerState; }
    public void setPower(PowerState state) {
        // return on same state
        if (powerState == state) return;

        powerState = state;
        // apply
        if (state == PowerState.ON) {
            intake1.setPower(INTAKE_POWER);
            intake2.setPower(INTAKE_POWER);
        } else if (state == PowerState.OFF) {
            intake1.setPower(0.0);
            intake2.setPower(0.0);
        }
    }
    public void togglePower() {
        setPower((getPower() == PowerState.ON) ? PowerState.OFF : PowerState.ON);
    }

    /* DIRECTION */
    public DirectionState getDirection() { return directionState; }
    public void setDirection(DirectionState state) {
        // return on same state
        if (directionState == state) return;

        directionState = state;
        // apply
        if (state == DirectionState.INTAKE) {
            intake1.setDirection(INTAKE_DIRECTION);
            intake2.setDirection(INTAKE_DIRECTION);
        } else if (state == DirectionState.OUTTAKE) {
            intake1.setDirection(OUTTAKE_DIRECTION);
            intake2.setDirection(OUTTAKE_DIRECTION);
        }
        // a direction change only reaches the motor on the next power write,
        // so resend power if the intake is already running
        if (powerState == PowerState.ON) {
            intake1.setPower(INTAKE_POWER);
            intake2.setPower(INTAKE_POWER);
        };
    }
    public void toggleDirection() {
        setDirection((getDirection() == DirectionState.INTAKE) ? DirectionState.OUTTAKE : DirectionState.INTAKE);
    }
}
