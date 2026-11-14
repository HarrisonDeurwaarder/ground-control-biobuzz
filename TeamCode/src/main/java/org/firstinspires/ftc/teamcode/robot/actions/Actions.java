package org.firstinspires.ftc.teamcode.robot.actions;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.robot.actions.fixed.ParallelAction;
import org.firstinspires.ftc.teamcode.robot.actions.fixed.RaceAction;
import org.firstinspires.ftc.teamcode.robot.actions.fixed.SequentialAction;

import java.util.function.BooleanSupplier;

public class Actions {
    private Actions() {}

    private static final Runnable NOOP = () -> {};
    private static final BooleanSupplier NOCOND = () -> true;

    /* SINGLE-ACTION STATIC FACTORIES */

    public static LambdaAction make(Runnable onStart, Runnable onStep, Runnable onStop, BooleanSupplier completionCondition) { return new LambdaAction(onStart, onStep, onStop, completionCondition); }
    public static LambdaAction make(Runnable onStart, Runnable onStep, BooleanSupplier completionCondition) { return new LambdaAction(onStart, onStep, NOOP, completionCondition); }
    public static LambdaAction make(Runnable onStep, BooleanSupplier completionCondition) { return new LambdaAction(NOOP, onStep, NOOP, completionCondition); }
    public static LambdaAction make(Runnable onStep) { return new LambdaAction(NOOP, onStep, NOOP, NOCOND); }
    public static LambdaAction doNothingAtAll() { return new LambdaAction(NOOP, NOOP, NOOP, NOCOND); }
    public static LambdaAction waitFor(BooleanSupplier completionCondition) { return new LambdaAction(NOOP, NOOP, NOOP, completionCondition); }

    // pure delay for X seconds (no actions)
    public static LambdaAction delay(double seconds) {
        // too lazy to not do this
        Timer dummyTimer = new Timer();
        return new LambdaAction(dummyTimer::reset, NOOP, NOOP, () -> dummyTimer.seconds() >= seconds);
    }

    // schedule a task to run every X seconds (when paired with a parallel action)
    public static LambdaAction schedule(Runnable task, double seconds) {
        Timer dummyTimer = new Timer();
        return new LambdaAction(
                () -> { dummyTimer.reset(); task.run(); }, NOOP, NOOP, () -> dummyTimer.seconds() >= seconds
        );
    }

    // run a task for X seconds
    public static LambdaAction doForSeconds(Runnable onStep, double seconds) {
        Timer dummyTimer = new Timer();
        return new LambdaAction(dummyTimer::reset, onStep, NOOP, () -> dummyTimer.seconds() >= seconds);
    }

    public static LambdaAction followPath(Follower follower, Path path) {
        return new LambdaAction(() -> follower.follow(path), NOOP, NOOP, follower::atParametricEnd);
    }

    /* COMPOSITE STATIC FACTORIES */

    public static SequentialAction sequential(Action... actions) { return new SequentialAction(actions); }
    public static ParallelAction parallel(Action... actions) { return new ParallelAction(actions); }
    public static RaceAction race(Action... actions) { return new RaceAction(actions); }
}
