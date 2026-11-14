package org.firstinspires.ftc.teamcode.robot.auto.utils;


import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.robot.actions.Action;
import org.firstinspires.ftc.teamcode.robot.actions.Actions;
import static org.firstinspires.ftc.teamcode.robot.actions.Actions.*;

class AutoTemplate extends AutoBase {

    public static Pose startingPose = new Pose(0.0, 0.0, Math.toRadians(90.0));
    public static Pose pose1 = new Pose(0.0, 0.0, Math.toRadians(45.0));
    public static Pose pose2 = new Pose(0.0, 0.0, Math.toRadians(-45.0));
    public static Pose defaultEndPose = new Pose(10.0, 0.0, Math.toRadians(0.0));

    protected Path toPose1, toPose2then1, toPose2, toEnd;

    @Override
    protected Pose getStartingPose() { return startingPose; }

    /*
    Unlike TeleOpTemplate, common OpMode inherited methods (init, start, loop, etc) should likely not be overridden.
    As such, they can be removed from this class.
     */
    @Override
    public void init() {
        super.init();
        // add extra init functionality below, or delete this method signature
        // ...
    }

    @Override
    public void init_loop() {
        super.init_loop();
        // add extra init_loop functionality below, or delete this method signature
        // ...
    }

    @Override
    public void start() {
        super.start();
        // add extra start functionality below, or delete this method signature
        // ...
    }

    @Override
    public void loop() {
        super.loop();
        // add extra loop functionality below, or delete this method signature
        // ...
    }

    @Override
    public void stop() {
        super.stop();
        // add extra stop functionality below, or delete this method signature
        // ...
    }

    protected void buildPaths() {
        // instantiate Path and PathChain objects here, using static poses defined in the class definition
        // these will be subsequently registered as cycles in registerPaths()
        toPose1 = Paths.line(startingPose, pose1).linear(startingPose, pose1);

        toPose2then1 = Paths.line(pose1, pose2).tangent();

        toPose2 = Paths.line(pose1, pose2).linear(pose1, pose2);

        toEnd = Paths.curve(pose2, pose1, defaultEndPose).tangent();
    }

    protected void initializeActions() {
        // here's an example auto
        // you will much more often use SequentialAction than any other composite
        // you can reuse the same action, but it must be cloned
        Action firstDelay = sequential(
                delay(3.0),
                followPath(follower, toPose1)
        );
        // set "actions" at the end
        actions = sequential(
                firstDelay.clone(),
                sequential(
                        delay(2.0),
                        doNothingAtAll(),
                        followPath(follower, toPose2then1),
                        followPath(follower, toPose2)
                ),
                firstDelay.clone(),
                sequential(
                        followPath(follower, toEnd)
                )
        );
    }
}