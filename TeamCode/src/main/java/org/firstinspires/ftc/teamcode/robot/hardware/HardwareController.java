package org.firstinspires.ftc.teamcode.robot.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.robot.hardware.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;

@Configurable
public class HardwareController {
    private Team team;
    private Intake intake;
    private Shooter shooter;

    // goal poses
    private Vector posRedGoal =  new Vector(0.0, 0.0);
    private Vector negRedGoal =  new Vector(0.0, 0.0);
    private Vector posBlueGoal = new Vector(0.0, 0.0);
    private Vector negBlueGoal = new Vector(0.0, 0.0);

    // declare actuators
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;

    // enums
    public enum Team { BLUE, RED }

    // trackers
    private Pose currentGoalPose;

    /**
     * Instantiate devices and set all to default configuration
     *
     * @param hardwareMap HardwareMap object
     */
    public HardwareController(HardwareMap hardwareMap, Team team) {
        this.team = team;

        /* DRIVETRAIN */
        leftFront =  hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack =   hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack =  hardwareMap.get(DcMotorEx.class, "rightBack");

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        setDefaults();
    }

    /* GETTERS AND SETTERS */

    public Intake intake() { return intake; }
    public Shooter shooter() { return shooter; }
    public Team team() { return team; }
    public Vector getGoal() {
        // without localization the field half is unknown, so pick by team only
        return (team == Team.RED) ? posRedGoal : posBlueGoal;
    }

    public void disable() {
        // DISABLE ALL MECHANISMS
        intake.disable(); shooter.disable();
    }

    /* HELPERS */

    /**
     * Set all devices to default configuration (direction, usage of encoder, etc)
     */
    private void setDefaults() {
        // DRIVETRAIN (directions match pedro/Constants.drivetrainConfig)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx motor : new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        // INTAKE
        intake.setDefaults();
        // SHOOTER
        shooter.setDefaults();
    }
}