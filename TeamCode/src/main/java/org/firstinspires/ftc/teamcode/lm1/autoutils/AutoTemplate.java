package org.firstinspires.ftc.teamcode.lm1.autoutils;


import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

class AutoTemplate extends AutoBase {

    protected static Pose startingPose = new Pose(0.0, 0.0, Math.toRadians(90.0));
    public static Pose pose1 = new Pose(0.0, 0.0, Math.toRadians(45.0));
    public static Pose pose2 = new Pose(0.0, 0.0, Math.toRadians(-45.0));
    public static Pose defaultEndPose = new Pose(10.0, 0.0, Math.toRadians(0.0));

    protected Path toPose1, toPose2then1, toPose2, toEnd;
    protected Cycle firstDelayCycle, secondDelayCycle, finalCycle;

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

    protected void registerPaths() {
        // register paths with AutoBuilder here
        // use Callbacks for executing runnables mid-path, but use AutoBuilder cycles for sequential auto building
        firstDelayCycle = new Cycle();
        firstDelayCycle.addDelay(3.0); // delay before following path
        firstDelayCycle.addFollowPedroPath(follower, toPose1);

        secondDelayCycle = new Cycle();
        secondDelayCycle.addDelay(2.0);
        secondDelayCycle.addPath(() -> {}, () -> true); // add an empty custom path that does nothing and continues immediately
        secondDelayCycle.addFollowPedroPath(follower, toPose2then1);
        secondDelayCycle.addFollowPedroPath(follower, toPose2);

        finalCycle = new Cycle();
        finalCycle.addFollowPedroPath(follower, toEnd);

        // add cycles to autobuilder
        autoBuilder.addCycle(firstDelayCycle);
        autoBuilder.addCycle(secondDelayCycle);
        autoBuilder.addCycle(firstDelayCycle); // the same cycle can be added twice (e.g. make one shoot cycle and add it several times)
        autoBuilder.addCycle(finalCycle);
    }
}