package org.firstinspires.ftc.teamcode.lm1.tests;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lm1.teleop.utils.TeleOpBase;

@TeleOp(name="Basic Drive", group="Test")
public class BasicDrive extends TeleOpBase {
    Pose startingPose = new Pose(0.0, 0.0);

    @Override
    public void init() {
        defaultStartingPose = startingPose;
        super.init();
    }

    @Override
    public void updateControls() {
        // no additional controls
    }

    @Override
    public void displayControls() {
        // no controls to display
    }
}
