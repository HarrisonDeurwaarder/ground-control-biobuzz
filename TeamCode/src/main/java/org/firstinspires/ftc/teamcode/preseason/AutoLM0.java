package org.firstinspires.ftc.teamcode.preseason;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.lm0.autoutils.AutoBase;
import org.firstinspires.ftc.teamcode.lm0.autoutils.Cycle;

/*
this code needs measurements to apply
? means this line of code needs measurements
 */
@Disabled
abstract class AutoLM0 extends AutoBase {

    protected static Pose startingPose = new Pose(12.0, 12.0, Math.toRadians(90.0)); // ?
    public static Pose shootingPose = new Pose(24.0, 36.0, Math.toRadians(45.0)); //Shooting pose, ?
    public static Pose intakestartPose = new Pose(48.0, 36.0, Math.toRadians(-45.0)); //Intake start pose, ?
    public static Pose intakeendPose = new Pose(10.0, 0.0, Math.toRadians(0.0)); //Intake end pose, ?
    public static Pose endPose = new Pose(40, 0, Math.toRadians(0.0)); // End pose, ?

    protected PathChain startingPath, intaketoshootPath, shootPath, tointakeendtoendPath, toendPath;
    protected Cycle firstDelayCycle, secondDelayCycle, finalCycle;

    @Override
    protected Pose getStartingPose() {
        return startingPose;
    }

    /*
    Unlike TeleOpTemplate, common OpMode inherited methods (init, start, loop, etc) should likely not be overridden.
    As such, they can be removed from this class.
     */

    protected void buildPaths() {
        // instantiate Path and PathChain objects here, using static poses defined in the class definition
        // these will be subsequently registered as cycles in registerPaths()
        startingPath = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, shootingPose)) // Bezier curves can be used for curved paths, which are often more efficient than lines if possible (retains velocity along curve)
                .setLinearHeadingInterpolation(startingPose.getHeading(), shootingPose.getHeading())
                .build(); //Starting pos -> shooting pos

        shootPath = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, intakestartPose))
                .setLinearHeadingInterpolation(
                        shootingPose.getHeading(),
                        intakestartPose.getHeading())
                .build(); //Shooting pos -> Intake start pos

        tointakeendtoendPath = follower.pathBuilder()
                .addPath(new BezierLine(intakestartPose, intakeendPose))
                .setLinearHeadingInterpolation(
                        intakestartPose.getHeading(),
                        intakeendPose.getHeading())
                .build(); //Intake start pos -> Intake end pos

        intaketoshootPath = follower.pathBuilder()
                .addPath(new BezierLine(intakeendPose, shootingPose))
                .setLinearHeadingInterpolation(
                        intakeendPose.getHeading(),
                        shootingPose.getHeading())
                .build();

        toendPath = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, endPose))
                .setLinearHeadingInterpolation(
                        shootingPose.getHeading(),
                        endPose.getHeading())
                .build();
    }


        protected void registerPaths() {
            // register paths with AutoBuilder here
            // use Callbacks for executing runnables mid-path, but use AutoBuilder cycles for sequential auto building

            // First shooting
            firstDelayCycle = new Cycle();
            firstDelayCycle.addFollowPedroPath(follower, startingPath);
            // Add shooting funcs here

            // Intake
            secondDelayCycle = new Cycle();
            secondDelayCycle.addFollowPedroPath(follower, shootPath);
            secondDelayCycle.addFollowPedroPath(follower, tointakeendtoendPath);
            secondDelayCycle.addDelay(1.0);

            // Return and second shooting
            finalCycle = new Cycle();
            finalCycle.addFollowPedroPath(follower, intaketoshootPath);
            finalCycle.addDelay(2.0);
            finalCycle.addFollowPedroPath(follower, toendPath);
            // add cycles to autobuilder
            autoBuilder.addCycle(firstDelayCycle);
            autoBuilder.addCycle(secondDelayCycle);
            autoBuilder.addCycle(finalCycle);
        }
    }
