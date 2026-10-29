package org.firstinspires.ftc.teamcode.lm1.hardwareutils;

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
import org.firstinspires.ftc.teamcode.lm1.hardwareutils.controllers.Intake;
import org.firstinspires.ftc.teamcode.lm1.hardwareutils.controllers.Shooter;

@Config
public class HardwareController {
    private Follower follower;
    private Team team;
    private Intake intake;
    private Shooter shooter;

    // declare actuators
    public DcMotorEx leftFront, leftBack, rightFront, rightBack, transfer, flywheel, intakeMotor;
    public ServoImplEx hood, flowerMech, intakeServo;
    public Limelight3A limelight;

    // enums
    public enum GateState { OPEN, CLOSED }
    public enum Team { BLUE, RED }

    // trackers
    public Pose currentGoalPose;

    /**
     * Instantiate devices and set all to default configuration
     *
     * @param hardwareMap HardwareMap object
     */
    public HardwareController(HardwareMap hardwareMap, Follower follower, Team team) {
        this.follower = follower;
        this.team = team;

        /* DRIVETRAIN */
        leftFront =  hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack =   hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack =  hardwareMap.get(DcMotorEx.class, "rightBack");

        /* LIMELIGHT */
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        /* MAP MECHANISM ACTUATORS */
        // GoBilda Yellowjackets
        transfer =    hardwareMap.get(DcMotorEx.class, "transfer");
        flywheel =    hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        // Axon MAX MK2
        flowerMech =  hardwareMap.get(ServoImplEx.class, "flowerMech");
        intakeServo = hardwareMap.get(ServoImplEx.class, "intakeServo");
        // Axon MINI MK2
        hood =        hardwareMap.get(ServoImplEx.class, "hood");

        setDefaults();
    }

    /* GETTERS AND SETTERS */

    public Intake getIntake() { return intake; }
    public Shooter getShooter() { return shooter; }
    public Team getTeam() { return team; }
    public void setTeam(Team team) { this.team = team; }

    /**
     * Set all devices to default configuration (direction, usage of encoder, etc)
     */
    private void setDefaults() {
        // LIMELIGHT
        limelight.setPollRateHz(100);
        limelight.start();

        // INTAKE
        intake.setDefaults();

        // SHOOTER
        shooter.setDefaults();

        // set device direction, encoder usage, and zero power behavior (i.e. braking) here
        // do not set drivetrain configuration; pedro will handle this
        // don't haphazardly modify drivetrain motors
        // ...

    }
}