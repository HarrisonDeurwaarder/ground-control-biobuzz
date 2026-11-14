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

    DcMotorEx intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        // map actuators used by the intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
    }

    /* SETTERS */

    public void setDefaults() {
        // INTAKE MOTOR
        setDirection(DirectionState.INTAKE);
        setPower(PowerState.OFF);
    }

    public void disable() {
        intakeMotor.setPower(0.0);
    }

    /* POWER */
    public PowerState getPower() { return powerState; }
    public void setPower(PowerState state) {
        powerState = state;
        // apply
        if (state == PowerState.ON) {
            intakeMotor.setPower(INTAKE_POWER);
        } else if (state == PowerState.OFF) {
            intakeMotor.setPower(0.0);
        }
    }
    public void togglePower() {
        setPower((getPower() == PowerState.ON) ? PowerState.OFF : PowerState.ON);
    }

    /* DIRECTION */
    public DirectionState getDirection() { return directionState; }
    public void setDirection(DirectionState state) {
        directionState = state;
        // apply
        if (state == DirectionState.INTAKE) {
            intakeMotor.setDirection(INTAKE_DIRECTION);
        } else if (state == DirectionState.OUTTAKE) {
            intakeMotor.setDirection(OUTTAKE_DIRECTION);
        }
    }
    public void toggleDirection() {
        setDirection((getDirection() == DirectionState.INTAKE) ? DirectionState.OUTTAKE : DirectionState.INTAKE);
    }
}
