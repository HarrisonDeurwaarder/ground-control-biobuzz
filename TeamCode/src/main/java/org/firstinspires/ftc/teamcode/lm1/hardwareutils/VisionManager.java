package org.firstinspires.ftc.teamcode.lm1.hardwareutils;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lm1.actions.Action;
import static org.firstinspires.ftc.teamcode.lm1.actions.Actions.*;

import java.util.ArrayList;


// IMPORTANT
// visit http://limelight.local:5801 (with the limelight AND your computer connected to the control hub)
// create a new pipeline for each color (indicies below) and set it to Color/Retroreflective
// configure the pipeline (again, for each color)
// the limelight is a USB device, not I2C

public class VisionManager {
    public static int HZ = 100;
    public static int POLLEN_IDX = 0;

    private Limelight3A limelight;
    private ArrayList<LLResultTypes.ColorResult> blobs;

    public VisionManager(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // configure the limelight
        limelight.setPollRateHz(HZ);
        limelight.pipelineSwitch(POLLEN_IDX);
        limelight.start();
    }

    public void update() {
        // fetch the latest result to unpack
        LLResult result = limelight.getLatestResult();
        // proceed on a substantive result
        if (result != null && result.isValid()) {
            result.get;
        }
    }

    class PollenDetection {
        private double x, y;
    }
}
