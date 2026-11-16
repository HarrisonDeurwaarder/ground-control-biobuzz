package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.core.actions.essential.Action;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.core.actions.essential.Actions;
import org.firstinspires.ftc.teamcode.robot.core.actions.essential.LambdaAction;
import org.firstinspires.ftc.teamcode.robot.core.math.Helpers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;

public class MissionControl {
    private static MissionControl instance;
    private ArrayList<Task> tasks;
    private Task active;

    // SHARED VARIABLES
    private long resetTimestamp, dt;
    private long lastTime = -1;
    private Follower follower;
    private HardwareMap hardwareMap;

    public enum ResetBehavior { DELETE, CLEAR, NONE }

    /* CONSTRUCTORS */

    private MissionControl() { /* needless constructor, but restricts access */ }



    /* GETTERS & SETTERS */

    public Task active() { return active; }
    public ArrayList<Task> tasks() { return tasks; }

    public HardwareMap hardwareMap() { return hardwareMap; } // getter
    public void hardwareMap(HardwareMap h) { hardwareMap = h; } // setter



    /* INTRINSIC VARIABLES */

    public long time() { return resetTimestamp; }
    public double dt() { return dt / 1e9; } // default to seconds
    public long dtMillis() { return TimeUnit.NANOSECONDS.toMillis(dt); }
    public long dtNano() { return dt; }



    /* FUNCTIONAL METHODS */

    public Task bind(Task task) { tasks.add(task); return task; }
    public Task bind(Action action) { Task task = new Task(action); tasks.add(task); return task; }
    public Task bind(Runnable runnable) {
        Task task = new Task(Actions.make(runnable)); tasks.add(task); return task;
    }

    public Follower follower() { // getter/maker
        if (follower == null) return new Follower(
                new PinpointLocalizer(hardwareMap, Constants.Pedro.pinpoint),
                new Mecanum(hardwareMap, Constants.Pedro.drivetrain),
                new Foresight(Constants.Pedro.foresight)
        );
        return follower;
    }

    public void reset() {
        resetTimestamp = System.nanoTime();
    }

    // to be called once in the OpMode loop
    public void tick() {
        // COMPUTE DT
        long time = System.nanoTime();
        dt = (lastTime < 0L) ? Constants.MissionControl.MIN_DT : time - lastTime;
        // clamp to safe ranges (prevents extreme updates)
        dt = Helpers.clamp(dt, Constants.MissionControl.MIN_DT, Constants.MissionControl.MAX_DT);
    }



    /* FACTORY */

    public static MissionControl instance() {
        // instantiate if null else return shared instance
        // only one instance of the class can exist
        if (instance == null) { instance = new MissionControl(); }
        return instance;
    }

    /* HELPERS */

    private void makeOrder() {

    }



    /* NESTED CLASSES */

    public static class Manifest {
        private final HashMap<String, Entry<?>> entries = new HashMap<>();
        private int index = -1; // for auto-naming

        public Manifest() {

        }

        /* HOLY NESTED CLASSES */

        // "T" compiles as whatever type the object is declared as (ex. Entry<String>)
        public class Entry<T> {
            private final String name; private final T fallback; private final boolean persistent;

            public Entry(String name, T fallback, boolean persistent) {
                this.name = name; this.fallback = fallback; this.persistent = persistent;
            }
            public Entry(T fallback, boolean persistent) { this(Constants.MissionControl.PREFIX + index++, fallback, persistent); }
            public Entry(String name, T fallback) { this(name, fallback, false); }
        }
    }

    public static class Task {
        private final Action action; // underlying action to execute
        private ResetBehavior onReset;

        /* CONSTRUCTORS */

        public Task(Action action, ResetBehavior onReset) {
            this.action = action; this.onReset = onReset;
        }
        public Task(Action action) { this(action, ResetBehavior.DELETE); }
        public Task(Runnable runnable, double latencySec, ResetBehavior onReset) {
            this.action = Actions.periodic(runnable, latencySec); this.onReset = onReset;
        }
        public Task(Runnable runnable, double latencySec) { this(runnable, latencySec, ResetBehavior.DELETE); }
        public Task(Runnable runnable, ResetBehavior onReset) {
            this.action = Actions.make(runnable); this.onReset = onReset;
        }
        public Task(Runnable runnable) { this(runnable, ResetBehavior.DELETE); }
    }
}
