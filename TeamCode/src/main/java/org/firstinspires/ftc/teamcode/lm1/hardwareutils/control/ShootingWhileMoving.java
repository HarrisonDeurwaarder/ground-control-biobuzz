package org.firstinspires.ftc.teamcode.lm1.hardwareutils.control;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.math.Velocity;

public class ShootingWhileMoving {

    double velocityLatency, rotationalLatency;

    public ShootingWhileMoving(double velocityLatency, double rotationalLatency) {
        this.velocityLatency = velocityLatency;
        this.rotationalLatency = rotationalLatency;
    }

    public Pose applyToPose(Pose robot, Vector turretOffset, Velocity velocity) {
        // turret pose application depends on robot heading
        Pose turretPose = robot.compose(
                turretOffset.toVector2D().toPose()
        );
        // apply swm
        return new Pose(
                turretPose.x() + velocityLatency * velocity.vx,
                turretPose.y() + velocityLatency * velocity.vy,
                turretPose.heading() + rotationalLatency * velocity.omega
        );
    }
}
