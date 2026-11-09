package org.firstinspires.ftc.teamcode.lm1.hardware.vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lm1.hardware.control.TimeDependentLPF;
import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;

import java.util.ArrayList;


// IMPORTANT
// visit http://limelight.local:5801 (with the limelight AND your computer connected to the control hub)
// create a new pipeline for each color (indicies below) and set it to Color/Retroreflective
// configure the pipeline (again, for each color)
// the limelight is a USB device, not I2C

public class VisionManager {
    public static int HZ = 100;
    public static int POLLEN_IDX = 0;
    public static boolean PREDICT_KINEMATICS = true;

    enum Type { POLLEN }

    private final Limelight3A limelight;
    private final Follower follower;
    private ArrayList<Sighting> sightings;

    public VisionManager(HardwareMap hardwareMap, Follower follower) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.follower = follower;
        // configure the limelight
        limelight.setPollRateHz(HZ);
        limelight.pipelineSwitch(POLLEN_IDX);
        limelight.start();
    }

    /* GETTERS */

    public Limelight3A limelight() { return limelight; }

    /* FUNCTIONAL METHODS */

    public void update() {
        // fetch the latest result to unpack
        LLResult result = limelight.getLatestResult();
        // proceed on a substantive result
        if (result != null && result.isValid()) {
            for (LLResultTypes.ColorResult detection : result.getColorResults()) matchDetection(detection);
        }
    }

    private void matchDetection(LLResultTypes.ColorResult detection) {
        // attempt to predict velocity + merge detections
        if (PREDICT_KINEMATICS) {
            // compare all detections to neighbors
            for (int i = 0; i < sightings.size(); i++) {

            }
        }
    }

    static class Sighting {
        private Vector p, v;
        private double confidence = 0.0; // neutral confidence score
        private int discreteAge = 0;
        private TimeDependentLPF lpfp = new TimeDependentLPF(1.0, 3), lpfv = new TimeDependentLPF(1.0, 3);
        private Timer timer = new Timer();

        // scratch objects
        private Vector zeroVect3d = Vector.zero(3);
        private Vector errorp3d = zeroVect3d.clone(), errorv3d = zeroVect3d.clone(), canv = zeroVect3d.clone(), canp = zeroVect3d.clone();
        private double errorp = 0.0, errorv = 0.0;

        public Sighting(double x, double y, double z) {
            p = new Vector(x, y, z);
            v = Vector.zero(3);
            lpfp.reset(); lpfv.reset();
        }

        /* GETTERS & SETTERS*/

        public double staleness() { return timer.seconds(); }

        public void update(Vector p) {
            // get dt
            double dt = timer.seconds();
            timer.reset();
            discreteAge += 1;

            // low-pass position results
            canp = lpfp.apply(p);
            // then compute velocity AND LOW PASS AGAIN (we love bias)
            Vector.sub(p, canp, canv);
            Vector.div(canv, dt, canv);
            // lp
            canv = lpfv.apply(canv);
            // compute measurement vs. filtered errors for confidence
            Vector.sub(p, canp, errorp3d); errorp = errorp3d.mag();
            Vector.sub(v, canv, errorv3d); errorv = errorv3d.mag();
            // push to permavalues
            p.from(canp); v.from(canv);
        }
    }
}
