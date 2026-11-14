package org.firstinspires.ftc.teamcode.robot.hardware.control;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.robot.math.linalg.Vector;

public class ShootingWhileMoving {
    SWMParameters parameters;
    Vector a = Vector.zero(3), b = Vector.zero(3);

    public ShootingWhileMoving(SWMParameters parameters) { this.parameters = parameters; }

    public Pose applyToPose(Pose robot, Vector turretOffset, Velocity velocity, double toF) {
        // turret pose application depends on robot heading
        a.set(robot.x(), robot.y(), 0.0);
        Vector.add(a, turretOffset, b);

        // apply swm
        return new Pose(
                b.at(0) - (parameters.linearLatency + toF) * velocity.vx,
                b.at(1) - (parameters.linearLatency + toF) * velocity.vy,
                robot.heading() - parameters.rotationalLatency * velocity.omega
        );
    }

    public static class SWMParameters {
        public double linearLatency, rotationalLatency;

        public SWMParameters(double linearLatency, double rotationalLatency) { this.linearLatency = linearLatency; this.rotationalLatency = rotationalLatency; }
    }
}
