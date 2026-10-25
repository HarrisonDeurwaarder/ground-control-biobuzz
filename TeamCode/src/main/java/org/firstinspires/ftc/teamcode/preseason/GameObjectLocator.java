package org.firstinspires.ftc.teamcode.preseason;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GameObjectLocator")
public class GameObjectLocator extends LinearOpMode {
    private static final double CAMERA_HEIGHT_IN = 10.0; // change after measurement
    private static final double BALL_RADIUS_IN = 2.8;
    private static final double CAMERA_PITCH_DEG = -10; // change after measurement

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight =
                hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                int count = 0;

                if (result != null && result.isValid()) {
                    for (LLResultTypes.DetectorResult ball :
                            result.getDetectorResults()) {
                        String color = ball.getClassName();
                        if (!"yellow".equalsIgnoreCase(color)) continue;

                        if (ball.getConfidence() < 0.7) continue;

                        double[] xy = calculatePosition(
                                ball.getTargetXDegreesNoCrosshair(),
                                ball.getTargetYDegreesNoCrosshair());

                        if (xy == null) continue;

                        count++;
                        telemetry.addData(
                                "Pollens" + count,
                                "right: %.1f in, forward: %.1f in",
                                xy[0], xy[1]);
                    }
                }

                telemetry.addData("Usable balls", count);
                telemetry.update();
                sleep(20);
            }
        } finally {
            limelight.stop();
        }
    }

    private double[] calculatePosition(double txDeg, double tyDeg) {
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);

        //camera ray: right, up, forward
        double right = Math.tan(Math.toRadians(txDeg));
        double up = Math.tan(Math.toRadians(tyDeg));

        //rotate ray for camera pitch
        double forward = Math.cos(pitch) - up * Math.sin(pitch);
        double vertical = Math.sin(pitch) + up * Math.cos(pitch);

        //Camera above ball center, ray point downward
        if (CAMERA_HEIGHT_IN <= BALL_RADIUS_IN ||
                vertical >= -0.001 || forward <= 0) {
            return null;
        }

        double scale =
                (BALL_RADIUS_IN - CAMERA_HEIGHT_IN) / vertical;
        double x = scale * right;
        double y = scale * forward;

        return new double[]{x, y};
    }
}


