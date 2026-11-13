package org.firstinspires.ftc.teamcode.lm1.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.lm1.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.lm1.hardware.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.lm1.hardware.vision.VisionManager;

@Config
public class HardwareController {
    private Follower follower;
    private Team team;
    private Intake intake;
    private Shooter shooter;

    // goal poses
    private Pose posRedGoal =  new Pose(0.0, 0.0);
    private Pose negRedGoal =  new Pose(0.0, 0.0);
    private Pose posBlueGoal = new Pose(0.0, 0.0);
    private Pose negBlueGoal = new Pose(0.0, 0.0);

    // declare actuators
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    // enums
    public enum Team { BLUE, RED }

    // trackers
    private Pose currentGoalPose;

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

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        setDefaults();
    }

    /* GETTERS AND SETTERS */

    public Intake intake() { return intake; }
    public Shooter shooter() { return shooter; }
    public Team team() { return team; }
    public Pose getGoal() {
        if (team == Team.RED && follower.pose().y() >= 0) { return posRedGoal; }
        else if (team == Team.RED && follower.pose().y() < 0) { return negRedGoal; }
        else if (team == Team.BLUE && follower.pose().y() >= 0) { return posBlueGoal; }
        else { return posRedGoal; }
    }

    /* HELPERS */

    /**
     * Set all devices to default configuration (direction, usage of encoder, etc)
     */
    private void setDefaults() {
        // INTAKE
        intake.setDefaults();
        // SHOOTER
        shooter.setDefaults();
    }
}