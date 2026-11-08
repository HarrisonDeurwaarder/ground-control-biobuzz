package org.firstinspires.ftc.teamcode.lm1.models.shooter;

import org.firstinspires.ftc.teamcode.lm1.math.equations.LinearEquation;
import org.firstinspires.ftc.teamcode.lm1.math.linalg.Vector;

public class BallisticModelSolver extends BallisticModel {
    public static double[] EXIT_ANGLE_CANDIDATES = {55.0, 60.0, 65.0}; // degrees
    public static double[] EXIT_VELOCITY_CANDIDATES = {10.0, 20.0}; // in/sec

    // velocity sampling scalers
    public static double THEORETICAL_MIN_SCALER = 0.95; // scales the theory minimum speed based on non-drag dynamics (for the true min)
    public static double THEORETICAL_MAX_SCALER = 1.3; // scales the theory minimum speed (for the true max)
    public static double PAST_MIN_SCALER = 0.8;
    public static double PAST_MAX_SCALER = 1.2;
    public static double LOWER_RANGE_EXTENDER = 0.97, UPPER_RANGE_EXTENDER = 1.03;
    public static double DT = 0.01; // seconds -> controls the accuracy/time tradeoff
    public static int MAX_SIM_ITERATIONS = 300; // prevents stalling sim loops
    public static int MAX_SOLVER_ITERATIONS = 100; // prevent nonconvergence
    public static double SOLVER_THRESHOLD = 1.0; // in
    public static double[] ENTRY_ANGLE_BOUNDS = {-90.0, 0.0}; // degrees -> reject solutions with an entry angle lower than this
    public static double OPTIMAL_ENTRY_ANGLE = -30.0;
    public static double ERROR_WEIGHT = 1.0;
    public static double ANGLE_WEIGHT = 0.0;
    public static double SPEED_WEIGHT = 0.0;

    // coefficients and force application
    public static final double APPLICATION_DIRECTION = 1.0; // exerts lift with backspin (+); drag with forward spin (-)

    private enum SamplingPhase { NONE, BRACKETING, REFINING }

    private final BallisticState state = BallisticState.invalid(); // default to returning an invalid state before any sim is run

    private double[] exitRadianCandidates, entryRadianBounds;
    private double optimalEntryRadian;
    private double dragCoef, magnusCoef;

    public BallisticModelSolver(LinearEquation velocityApproximator, double dragCoef, double magnusCoef) {
        super(velocityApproximator); this.dragCoef = dragCoef; this.magnusCoef = magnusCoef;
    }

    public BallisticModelSolver(LinearEquation velocityApproximator, double dragCoef) { this(velocityApproximator, dragCoef, 0.0); }

    public BallisticModelSolver(LinearEquation velocityApproximator) { this(velocityApproximator, 0.0, 0.0); }

    /* GETTERS */

    public BallisticState state() { return state; }

    /* FUNCTIONAL METHODS */

    public void compute(double dx, double dy, double dz) {
        computeConversions();
        // "r" is the horizontal component in the direction of (dx, dy)--i.e. the shooting plane
        // we assume the turret is locked on
        double r = Math.hypot(dx, dy);
        // base vector reused a lot
        Vector zeroVector2d = Vector.zero(2);

        // INSTANTIATE ALL OBJECTS AS TO AVOID IT IN LOOPS
        Vector p = zeroVector2d.clone(), v = zeroVector2d.clone(), a = zeroVector2d.clone(), drag = zeroVector2d.clone(), magnus = zeroVector2d.clone(), vNormal = zeroVector2d.clone(), dp = zeroVector2d.clone();
        Vector grav = new Vector(0.0, -G_IN);

        // same with states
        BallisticState candState = new BallisticState();
        BallisticState[] transientStates = new BallisticState[EXIT_ANGLE_CANDIDATES.length];
        // we need transient states to evaluate angle candidates
        // fill with empty states
        for (int i = 0; i < transientStates.length; i++) { transientStates[i] = new BallisticState(); }

        SamplingData vSamData = new SamplingData();

        /* THETA LOOP */
        for (int i = 0; i < exitRadianCandidates.length; i++) {
            vSamData.reset();
            // run sim with this exit angle
            // velocity loop
            double exitAngle = exitRadianCandidates[i];

            // tangential velocity candidates
            // save error alongside for guess scaling
            // methods using past 2 samples are supported

            /* TAN VEL LOOP */
            for (int j = 0; true; j++) {
                // get scalar exit velocity from tangential velocity
                double exitVelocity = sampleCandidate(vSamData, r, dz, exitAngle);
                // no need to convert tangential to exit---endpoints are sampled in exit space

                // sim parameters
                p.from(zeroVector2d); // position in the exit frame
                v.set(exitVelocity * Math.cos(exitAngle), exitVelocity * Math.sin(exitAngle));
                a.set(0.0, -G_IN); // begin acceleration without drag

                /* SIMULATION LOOP */
                double lastX = 0.0; double lastZ = 0.0;
                double slope, intercept, error; double estZ = 0.0;
                // check the sim value of [t_exit, v_tan]
                int k = 0;
                error = dz; // true at t=0
                while (true) {
                    // initialize as zero vector
                    drag.from(zeroVector2d); magnus.from(zeroVector2d);

                    /* COMPUTE VELOCITY-DEPENDENT FORCES */

                    // compute drag magnitude
                    double mag = v.mag();
                    if (dragCoef != 0.0) Vector.mul(v, dragCoef * mag, drag);

                    if (magnusCoef != 0.0) {
                        // magnus generates perpendicular acceleration
                        Vector.normal(v, vNormal);
                        // reverse perpendicular direction to correct sign
                        if (Math.signum(vNormal.at(1)) != APPLICATION_DIRECTION)
                            vNormal.neg(); // in-place

                        // vector needs to be in the direction given by spin
                        // backspin = thrust; forward spin = drag
                        Vector.mul(vNormal, magnusCoef * mag, magnus);
                    }

                    // update kinematic parameters
                    a.from(magnus);
                    a.sub(drag).sub(grav);
                    v.add(a.mul(DT)); // we edit an in-place to avoid allocating new objects---it's overridden prior anyway
                    p.add(Vector.mul(v, DT, dp)); // we need to retain v, so we allocate a new Vector in the process

                    /* CHECK STOPPING CONDITION */
                    // by checking horizontal distance instead of vertical, we know the sim must converge
                    if (p.at(0) >= r) { // stop if beyond goal distance
                        // interpolate the true collision value
                        // z(x)
                        slope = (p.at(1) - lastZ) / (p.at(0) - lastX);
                        intercept = p.at(1) - slope * p.at(0);
                        // get estimated value (intercept with h=dz)
                        estZ = slope * r + intercept;

                        /* UPDATE ENDPOINT */
                        error = estZ - dz;
                        int index = error > 0.0 ? 0 : 1;

                        // set correct endpoint
                        if (index == 0) { vSamData.lo = exitVelocity; vSamData.loE = error; }
                        else vSamData.hi = exitVelocity; vSamData.hiE = error;
                        // apply the illinois trick
                        // if candidate velocities hit the same error-sign twice, halve the stuck side
                        // the OTHER side
                        if (index == vSamData.activeIndex && vSamData.phase == SamplingPhase.REFINING) {
                            // pull away from the stuck side
                            if (index == 0) vSamData.hiE /= 2.0;
                            else vSamData.loE /= 2.0;
                        }
                        vSamData.activeIndex = index;
                        break;
                    }

                    k++;
                    /* CHECK STALL */
                    // if it stalls, we automatically know it's an invalid state
                    if (k >= MAX_SIM_ITERATIONS) break;

                    // reset last parameters
                    lastX = p.at(0); lastZ = p.at(1);
                }

                // skip
                /* CHECK V_TAN CONVERGENCE */
                if (Math.abs(dz - estZ) <= SOLVER_THRESHOLD) {
                    transientStates[i].update(exitVelocity, velApx.inverse(exitVelocity), exitAngle, Math.atan2(v.at(1), v.at(0)), error, true);
                    break; // move on to the next angle
                }
                /* CHECK NONCONVERGENCE */
                if (j >= MAX_SOLVER_ITERATIONS || Double.isNaN(vSamData.hi) && Double.isNaN(vSamData.lo)) { // if an angle is infeasible, prevent NaN from propagating and causing stall
                    // return an invalid state
                    transientStates[i].updateInvalid();
                    break;
                }
            }
        }

        // accept the best state
        candState.update(transientStates[0]);
        // iteratively select/discard
        for (int j = 1; j < transientStates.length; j++){
            // update if entry angle is fine AND state is more advantageous
            if (valueInBounds(transientStates[j].entryTheta(), entryRadianBounds) && compareStates(transientStates[j], candState)) candState.update(transientStates[j]);
        }
        // now the user can pull "state" for shot prediction
        // arguments should already be passed with SwM applied
        state.update(candState);
    }

    /* HELPERS */

    private double sampleCandidate(SamplingData data, double r, double dz, double exitRadians) {

        /* SET SAMPLING PHASE */

        // begin bracketing not when candidate lo/hi are in place, but when they have been TESTED too
        if (Double.isFinite(data.loE) && Double.isFinite(data.hiE)) data.phase = SamplingPhase.BRACKETING; // establishing a valid bracket by expansion
        // begin refining when a valid negative-positive interval has been established
        if (data.loE <= 0.0 && data.hiE >= 0.0) data.phase = SamplingPhase.REFINING; // bringing error closer to zero

        /* ACT ON PHASE */

        // no bounds established yet
        if (data.phase == SamplingPhase.NONE) {
            // get candidate bounds first, then place as needed
            double canLo, canHi;
            if (state.valid()) { // sample from the last used state if it exists
                double pastVel = state.exitVelocity();
                canLo = PAST_MIN_SCALER * pastVel; canHi = PAST_MAX_SCALER * pastVel;
            } else { // otherwise, sample from closed-form approx
                double theory = Math.sqrt((G_IN * r * r) / (2 * Math.pow(Math.cos(exitRadians), 2) * (r * Math.tan(exitRadians) - dz)));
                canLo = THEORETICAL_MIN_SCALER * theory; canHi = THEORETICAL_MAX_SCALER * theory;
            }

            // do lower first
            if (Double.isNaN(data.lo)) { return canLo; }
            // then do upper
            else { return canHi; }
            // after both are run, the bounds should be established + sampled on the next iteration
        }

        // bounds established, but they need bracketing
        else if (data.phase == SamplingPhase.BRACKETING) {
            // do lower first
            if (data.loE < 0.0) { data.lo *= LOWER_RANGE_EXTENDER; return data.lo; }
            // then do upper
            else { data.hi *= UPPER_RANGE_EXTENDER; return data.hi; }
        }

        // bracketed and we can refine until in threshold
        // here, we must rely on error to place candidates
        // hence the simulation loop must do it
        else { return regulaFalsi(data); }

        // sim loop will SET CORRECT INDEX based on error
    }

    private boolean compareStates(BallisticState curState, BallisticState canState) {
        // accept a lesser error if it's inside the angle bounds
        double diffError, diffAngle, diffSpeed;

        // immediately reject invalid candidates OR currents
        if (!curState.valid()) { return true; }
        else if (!canState.valid()) { return false; }

        // establish a score weighing 1) proximity to the optimal entry angle
        // 2) minimizing error
        // 3) minimizing tangential speed changing
        diffError = Math.abs(curState.error()) - Math.abs(canState.error());
        diffAngle = Math.abs(optimalEntryRadian - curState.entryTheta()) - Math.abs(optimalEntryRadian - canState.entryTheta());
        diffSpeed = Math.abs(state.exitVelocity() - curState.exitVelocity()) - Math.abs(state.exitVelocity() - canState.exitVelocity()); // current velocity cancels

        // if the score is positive, we say state2. if its negative, then state1
        double score = ERROR_WEIGHT * diffError + ANGLE_WEIGHT * diffAngle + SPEED_WEIGHT * diffSpeed;
        return score > 0.0;
    }

    private static double regulaFalsi(SamplingData data) {
        // compute the root
        // since this is a linear APPROXIMATION, it won't be perfect
        // but it will ideally converge
        return data.lo - data.loE * (data.hi - data.lo) / (data.hiE - data.loE);
    }

    @Override
    protected void computeConversions() {
        exitRadianCandidates = new double[EXIT_ANGLE_CANDIDATES.length];
        entryRadianBounds = new double[ENTRY_ANGLE_BOUNDS.length];

        for (int i = 0; i < exitRadianCandidates.length; i++) exitRadianCandidates[i] = Math.toRadians(EXIT_ANGLE_CANDIDATES[i]);
        for (int i = 0; i < entryRadianBounds.length; i++) entryRadianBounds[i] = Math.toRadians(ENTRY_ANGLE_BOUNDS[i]);
        optimalEntryRadian = Math.toRadians(OPTIMAL_ENTRY_ANGLE);
    }

    private boolean valueInBounds(double value, double[] inclusiveBounds) { return inclusiveBounds[0] <= value && value <= inclusiveBounds[1]; }

    /* NESTED CLASSES */

    // pure data class
    static class SamplingData {
        double lo, hi, loE, hiE; int activeIndex; SamplingPhase phase;
        SamplingData() { reset(); }
        void reset() {
            double lo = Double.NaN, hi = Double.NaN, loE = Double.NaN, hiE = Double.NaN;
            int activeIndex = -1;
            SamplingPhase phase = SamplingPhase.NONE;
        }
    }
}
