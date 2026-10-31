package org.firstinspires.ftc.teamcode.lm1.hardwareutils.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.lm1.math.Helpers;

public class Intake {

    public static double FLOWER_UP_DEGREES = 0.0;
    public static double FLOWER_DOWN_DEGREES = 0.0;
    public static double FLOWER_GEARING = 1.0;
    public static double INTAKE_UP_DEGREES = 0.0;
    public static double INTAKE_POLLEN_DEGREES = 0.0;
    public static double INTAKE_DOWN_DEGREES = 0.0;
    public static double INTAKE_GEARING = 1.0;

    public static double INTAKE_POWER = 0.8;
    public static DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction OUTTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public enum    FlowerState { UP, DOWN }
    public enum  PositionState { UP, POLLEN, DOWN }
    public enum     PowerState { ON, OFF }
    public enum DirectionState { INTAKE, OUTTAKE }

    public FlowerState       flowerState = FlowerState.UP;
    public PositionState   positionState = PositionState.UP;
    public PowerState         powerState = PowerState.OFF;
    public DirectionState directionState = DirectionState.INTAKE;

    DcMotorEx intakeMotor;
    ServoImplEx intakeServo, flower;

    public Intake(HardwareMap hardwareMap) {
        // map actuators used by the intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeServo = hardwareMap.get(ServoImplEx.class, "intakeServo");
        flower = hardwareMap.get(ServoImplEx.class, "flower");
    }

    public void setDefaults() {
        // INTAKE MOTOR
        setDirection(DirectionState.INTAKE);
        setPower(PowerState.OFF);

        // INTAKE SERVO
        intakeServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        setIntakePosition(PositionState.UP);

        // FLOWER SERVO
        flower.setPwmRange(new PwmControl.PwmRange(500, 2500));
        setFlowerPosition(FlowerState.UP);
    }

    /* FLOWER */
    public FlowerState getFlowerPosition() { return flowerState; }
    public void setFlowerPosition(FlowerState state) {
        flowerState = state;
        // also apply position update
        if (state == FlowerState.UP) {
            setFlowerDegrees(FLOWER_UP_DEGREES);
        } else if (state == FlowerState.DOWN) {
            setFlowerDegrees(FLOWER_DOWN_DEGREES);
        }
    }
    public void toggleFlower() {
        setFlowerPosition((getFlowerPosition() == FlowerState.UP) ? FlowerState.DOWN : FlowerState.UP);
    }

    /* POSITION */
    public PositionState getIntakePosition() { return positionState; }
    public void setIntakePosition(PositionState state) {
        positionState = state;
        // apply
        if (state == PositionState.UP) {
            setIntakeDegrees(INTAKE_UP_DEGREES);
        } else if (state == PositionState.POLLEN) {
            setIntakeDegrees(INTAKE_UP_DEGREES);
        } else if (state == PositionState.DOWN) {
            setIntakeDegrees(INTAKE_DOWN_DEGREES);
        }
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

    /*
    * INTERNAL METHODS
    */

    private double getFlowerDegrees() {
        return flower.getPosition() * 350.0 / FLOWER_GEARING;
    }

    private void setFlowerDegrees(double degrees) {
        // preprocess degrees
        degrees = Helpers.boundDegrees(degrees * FLOWER_GEARING); // convert to output degrees
        degrees = Helpers.snapDegreesAxon(degrees);
        flower.setPosition(degrees / 350.0);
    }

    private double getIntakeDegrees() {
        return intakeServo.getPosition() * 350.0 / INTAKE_GEARING;
    }

    private void setIntakeDegrees(double degrees) {
        // preprocess degrees
        degrees = Helpers.boundDegrees(degrees * INTAKE_GEARING); // convert to output degrees
        degrees = Helpers.snapDegreesAxon(degrees);
        intakeServo.setPosition(degrees / 350.0);
    }
}
