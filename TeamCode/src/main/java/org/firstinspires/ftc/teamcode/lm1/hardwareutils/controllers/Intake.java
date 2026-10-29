package org.firstinspires.ftc.teamcode.lm1.hardwareutils.controllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.lm1.hardwareutils.Helpers;

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

    DcMotorEx intake;
    ServoImplEx intakePos, flowerPos;

    public Intake(DcMotorEx intake, ServoImplEx intakePos, ServoImplEx flowerPos) {
        // map actuators used by the intake
        this.intake = intake; this.intakePos = intakePos; this.flowerPos = flowerPos;
    }

    public void setDefaults() {
        // INTAKE MOTOR
        setPosition(PositionState.UP);
        setDirection(DirectionState.INTAKE);
        // INTAKE SERVO
        intakePos.setPwmRange(new PwmControl.PwmRange(500, 2500));
        // FLOWER SERVO
        flowerPos.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    /* FLOWER */
    public FlowerState getFlower() { return flowerState; }
    public void setFlower(FlowerState state) {
        flowerState = state;
        // also apply position update
        if (state == FlowerState.UP) {

        }
    }
    public void toggleFlower() {
        setFlower((getFlower() == FlowerState.UP) ? FlowerState.DOWN : FlowerState.UP);
    }

    /* POSITION */
    public PositionState getPosition() { return positionState; }
    public void setPosition(PositionState state) { positionState = state; }

    /* POWER */
    public PowerState getPower() { return powerState; }
    public void setPower(PowerState state) {
        powerState = state;
        // apply
        if (state == PowerState.ON) {
            intake.setPower(INTAKE_POWER);
        } else if (state == PowerState.OFF) {
            intake.setPower(0.0);
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
            intake.setDirection(INTAKE_DIRECTION);
        } else if (state == DirectionState.OUTTAKE) {
            intake.setDirection(OUTTAKE_DIRECTION);
        }
    }
    public void toggleDirection() {
        setDirection((getDirection() == DirectionState.INTAKE) ? DirectionState.OUTTAKE : DirectionState.INTAKE);
    }

    /*
    * INTERNAL METHODS
    */

    private double getFlowerDegrees() {
        return flowerPos.getPosition() * 350.0 / FLOWER_GEARING;
    }

    private void setFlowerDegrees(double degrees) {
        // preprocess degrees
        degrees = Helpers.boundDegrees(degrees * FLOWER_GEARING); // convert to output degrees
        degrees = Helpers.snapDegreesAxon(degrees);
        flowerPos.setPosition(degrees / 350.0);
    }

    private double getIntakeDegrees() {
        return intakePos.getPosition() * 350.0 / INTAKE_GEARING;
    }

    private void setIntakeDegrees(double degrees) {
        // preprocess degrees
        degrees = Helpers.boundDegrees(degrees * INTAKE_GEARING); // convert to output degrees
        degrees = Helpers.snapDegreesAxon(degrees);
        intakePos.setPosition(degrees / 350.0);
    }
}
