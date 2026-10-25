package org.firstinspires.ftc.teamcode.lm0.hardwareutils;

import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.BLUE_NEGATIVE_GOAL;
import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.BLUE_POSITIVE_GOAL;
import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.FLYWHEEL_DEFAULT_RPM;
import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.GATE_CLOSED;
import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.GATE_OPEN;
import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.GOAL_UPDATE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.HOOD_DEFAULT_ANGLE;
import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.RED_NEGATIVE_GOAL;
import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.RED_POSITIVE_GOAL;
import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.TURRET_OFFSET;
import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.flywheelInterp;
import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.flywheelPID;
import static org.firstinspires.ftc.teamcode.lm0.hardwareutils.HardwareConstants.hoodInterp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class HardwareController {
    public Follower follower;
    public Team team;

    // declare actuators
    public DcMotorEx leftFront, leftBack, rightFront, rightBack, flywheel, intakeFront, intakeBack;
    public ServoImplEx hood, gate, flowerMech, intakeServo;
    public Limelight3A limelight;

    // enums
    public enum GateState { OPEN, CLOSED }
    public enum FlowerState { UP, DOWN }
    public enum IntakeState { UP, DOWN, ON, OFF, INTAKE, OUTTAKE }
    public enum Team { BLUE, RED }

    // trackers
    public Pose currentGoalPose;
    public boolean flywheelActive, hoodActive, rotActive;

    /**
     * Instantiate devices and set all to default configuration
     *
     * @param hardwareMap HardwareMap object
     */
    public HardwareController(HardwareMap hardwareMap, Follower follower, Team team) {
        // map drivetrain motors
        leftFront =  hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack =   hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack =  hardwareMap.get(DcMotorEx.class, "rightBack");

        // map limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.follower = follower;
        this.team = team;

        // map mechanism actuators

        // motors
        flywheel =    hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");
        intakeBack =  hardwareMap.get(DcMotorEx.class, "intakeBack");
        // axons
        //turretRot1 =  hardwareMap.get(ServoImplEx.class, "turretRot1");
        //turretRot2 =  hardwareMap.get(ServoImplEx.class, "turretRot2");
        hood =        hardwareMap.get(ServoImplEx.class, "hood");
        gate =        hardwareMap.get(ServoImplEx.class, "gate");
        flowerMech =  hardwareMap.get(ServoImplEx.class, "flowerMech");
        intakeServo = hardwareMap.get(ServoImplEx.class, "intakeServo");

        // add more devices as needed
        // ...

        setAllToDefault();
    }

    /**
     * Set all devices to default configuration (direction, usage of encoder, etc)
     */
    private void setAllToDefault() {
        // set poll rate
        limelight.setPollRateHz(100);
        limelight.start();

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        setFlywheelSpeed(0.0);

        setIntakeState(IntakeState.UP);
        setIntakeState(IntakeState.OFF);
        setIntakeState(IntakeState.INTAKE);

        setFlowerState(FlowerState.UP);

        setGateState(GateState.CLOSED);

        //turretRot1.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axons use a different PWM range than GoBildas
        //turretRot2.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axons use a different PWM range than GoBildas
        //setTurretRotation(ROT_DEFAULT_ANGLE); // assign default

        hood.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axons use a different PWM range than GoBildas
        setHoodAngle(HOOD_DEFAULT_ANGLE); // assign default

        flowerMech.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axons use a different PWM range than GoBildas
        setFlowerState(FlowerState.UP);

        intakeServo.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axons use a different PWM range than GoBildas
        setIntakeState(IntakeState.UP);

        updateGoalPose();

        flywheelActive = true; hoodActive = true;

        // set device direction, encoder usage, and zero power behavior (i.e. braking) here
        // do not set drivetrain configuration; pedro will handle this
        // don't haphazardly modify drivetrain motors
        // ...

    }

    public void updateTurret() {
        /* first compute the respective poses */

        // apply shooting-while-moving
        //Pose correctedGoalPose = applySwM(currentGoalPose);
        // compute turret pose as a function of robot pose
        Pose turretPose = follower.pose().plus(TURRET_OFFSET);
        // compute delta pose
        Pose delta = currentGoalPose.minus(turretPose);
        double distance = delta.toVector2D().magnitude();

        /* compute and apply turret rotation*/

        /*double rotAngle;
        if (rotActive) {4
            // compute angle of delta pose
            rotAngle = Math.toDegrees(
                    Math.atan2(delta.getY(), delta.getX()) + delta.getHeading()
            );
        } else {
            rotAngle = ROT_DEFAULT_ANGLE;
        }
        // set as turret goal rotation
        setTurretRotation(rotAngle);*/

        /* compute and apply hood angle */

        double hoodAngle;
        if (hoodActive) {
            hoodAngle = hoodInterp.infer(distance);
        }
        else {
            hoodAngle = HOOD_DEFAULT_ANGLE;
        }
        setHoodAngle(hoodAngle);

        /* compute and apply flywheel regression */

        double targetVelocity;
        if (flywheelActive) {
            targetVelocity = flywheelInterp.infer(distance);
        }
        else {
            targetVelocity = FLYWHEEL_DEFAULT_RPM;
        }
        setFlywheelSpeed(targetVelocity);

        // apply pid
        flywheel.setPower(flywheelPID.compute(getFlywheelSpeed()));
    }

    public void setGateState(GateState state) {
        if (state == GateState.OPEN) {
            gate.setPosition(GATE_OPEN);
        } else if (state == GateState.CLOSED) {
            gate.setPosition(GATE_CLOSED);
        }
    }

    public void setIntakeState(IntakeState state) {
        // power
        if (state == IntakeState.ON) {
            intakeFront.setPower(HardwareConstants.INTAKE_POWER);
            intakeBack.setPower(HardwareConstants.INTAKE_POWER);
        } else if (state == IntakeState.OFF) {
            intakeFront.setPower(0.0);
            intakeBack.setPower(0.0);
        }

        // position
        else if (state == IntakeState.UP) {
            intakeServo.setPosition(HardwareConstants.INTAKE_UP);
        } else if (state == IntakeState.DOWN) {
            intakeServo.setPosition(HardwareConstants.INTAKE_DOWN);
        }

        // direction
        else if (state == IntakeState.INTAKE) {
            intakeFront.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeBack.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (state == IntakeState.OUTTAKE) {
            intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void setFlowerState(FlowerState state) {
        if (state == FlowerState.UP) {
            flowerMech.setPosition(HardwareConstants.FLOWER_MECH_UP);
        } else if (state == FlowerState.DOWN) {
            flowerMech.setPosition(HardwareConstants.FLOWER_MECH_DOWN);
        }
    }

    public double getHoodAngle() {
        double pos = hood.getPosition();
        // undo the degrees -> ratio comp
        return pos / HardwareConstants.TURRET_HOOD_RATIO + HardwareConstants.TURRET_HOOD_START_DEG;
    }

    public void setHoodAngle(double degrees) {
        degrees = boundDegrees(degrees);
        degrees = snapDegreesToAxonRange(degrees);
        // apply ratio
        double degreesCmd = (degrees - HardwareConstants.TURRET_HOOD_START_DEG) * HardwareConstants.TURRET_HOOD_RATIO;
        hood.setPosition(degreesCmd);
    }

    public double getFlywheelSpeed() {
        // get ticks-per-second value
        double radps = flywheel.getVelocity(AngleUnit.RADIANS);
        // return rpm
        return radps * 30.0 / Math.PI;
    }

    public void setFlywheelSpeed(double rpm) {
        // compute radps
        double radps = rpm * Math.PI / 30.0;
        flywheelPID.changeSetpoint(radps);
    }

    /*public double getTurretRotation() {
        double rot1 = getTurretRotation1();
        double rot2 = getTurretRotation2();
        // aggregate (avg)
        return (rot1 + rot2) / 2.0;
    }

    public double getTurretRotation1() {
        // bounded measurement 0-1
        double rot = turretRot1.getPosition();
        // convert to degrees
        return rot * 350.0;
    }

    public double getTurretRotation2() {
        // bounded measurement 0-1
        double rot = turretRot2.getPosition();
        // convert to degrees
        return rot * 350.0;
    }

    public void setTurretRotation(double degrees) {
        // transform degrees for safety
        degrees = boundDegrees(degrees);
        degrees = snapDegreesToAxonRange(degrees);

        double command = degrees / 350; // normalizes command 0-1
        // send PMW command
        turretRot1.setPosition(command);
        turretRot2.setPosition(command);
    }*/

    public void updateGoalPose() {
        // auto aim at whichever goal is in reach
        if (team == Team.BLUE && follower.pose().y() >= GOAL_UPDATE_THRESHOLD) {
            currentGoalPose = BLUE_POSITIVE_GOAL;

        } else if (team == Team.BLUE && follower.pose().y() < -GOAL_UPDATE_THRESHOLD) {
            currentGoalPose = BLUE_NEGATIVE_GOAL;

        } else if (team == Team.RED && follower.pose().y() >= GOAL_UPDATE_THRESHOLD) {
            currentGoalPose = RED_POSITIVE_GOAL;

        } else if (team == Team.RED && follower.pose().y() < -GOAL_UPDATE_THRESHOLD) {
            currentGoalPose = RED_NEGATIVE_GOAL;
        }
    }

    /*private Pose applySwM(Pose goalPose)  {
        Pose movementCorrector = new Pose(
                useTangentialCorrection ? follower.getVelocity().getXComponent() * VELOCITY_LATENCY + 0.5 * follower.getAcceleration().getXComponent() * Math.pow(ACCELERATION_LATENCY, 2) : 0.0,
                useTangentialCorrection ? follower.getVelocity().getYComponent() * VELOCITY_LATENCY + 0.5 * follower.getAcceleration().getYComponent() * Math.pow(ACCELERATION_LATENCY, 2) : 0.0,
                useRotationalCorrection ? follower.getAngularVelocity() * OMEGA_LATENCY : 0.0
        );

        return goalPose.minus(movementCorrector);
    }*/

    private double boundDegrees(double degrees) {
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

    private double snapDegreesToAxonRange(double degrees) {
        // if between 350-360, snap to nearest
        if (degrees > 350 && degrees < 355) {
            degrees = 350;
        } else if (degrees < 360 && degrees >= 355) {
            degrees = 0;
        }
        return degrees;
    }
}