package org.firstinspires.ftc.teamcode.lm1.actions;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.lm1.actions.fixed.ParallelAction;
import org.firstinspires.ftc.teamcode.lm1.actions.fixed.RaceAction;
import org.firstinspires.ftc.teamcode.lm1.actions.fixed.SequentialAction;

import java.util.function.BooleanSupplier;

public class Actions {
    private Actions() {}

    private static final Runnable NOOP = () -> {};
    private static final BooleanSupplier NOCOND = () -> true;

    /* SINGLE-ACTION STATIC FACTORIES */

    public static Action make(Runnable onStart, Runnable onStep, BooleanSupplier completionCondition) { return new LambdaAction(onStart, onStep, completionCondition); }
    public static Action make(Runnable onStep, BooleanSupplier completionCondition) { return new LambdaAction(NOOP, onStep, completionCondition); }
    public static Action make(Runnable onStep) { return new LambdaAction(NOOP, onStep, NOCOND); }
    public static Action doNothingAtAll() { return new LambdaAction(NOOP, NOOP, NOCOND); }
    public static Action waitFor(BooleanSupplier completionCondition) { return new LambdaAction(NOOP, NOOP, completionCondition); }
    public static Action delay(double seconds) {
        // too damn lazy to not do this
        Timer dummyTimer = new Timer();
        return new LambdaAction(dummyTimer::reset, NOOP, () -> dummyTimer.seconds() >= seconds);
    }

    public static Action doForSeconds(Runnable onStep, double seconds) {
        // too damn lazy to not do this
        Timer dummyTimer = new Timer();
        return new LambdaAction(dummyTimer::reset, onStep, () -> dummyTimer.seconds() >= seconds);
    }

    public static Action followPath(Follower follower, Path path) {
        return new LambdaAction(() -> follower.follow(path), NOOP, follower::atParametricEnd);
    }

    /* COMPOSITE STATIC FACTORIES */

    public static Action sequential(Action... actions) { return new SequentialAction(actions); }
    public static Action parallel(Action... actions) { return new ParallelAction(actions); }
    public static Action race(Action... actions) { return new RaceAction(actions); }
}
