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
    public static double DT = 0.1; // seconds -> controls the accuracy/time tradeoff
    public static int MAX_SIM_ITERATIONS = 100; // prevents stalling sim loops
    public static double MAX_SOLVER_ITERATIONS = 100; // prevent nonconvergence
    public static double SOLVER_THRESHOLD = 1.0; // in
    public static double[] ENTRY_ANGLE_BOUNDS = {-90.0, 0.0}; // degrees -> reject solutions with an entry angle lower than this
    public static double OPTIMAL_ENTRY_ANGLE = -30.0;
    public static double ERROR_WEIGHT = 1.0;
    public static double ANGLE_WEIGHT = 0.0;
    public static double SPEED_WEIGHT = 0.0;

    // coefficients and force application
    public static final double APPLICATION_DIRECTION = 1.0; // exerts lift with backspin (+); drag with forward spin (-)


    private final BallisticState state = BallisticState.invalid(); // default to returning an invalid state before any sim is run
    private final BallisticState[] transientStates = new BallisticState[EXIT_ANGLE_CANDIDATES.length];
    private double lastEndpoint;

    private double[] exitRadianCandidates = new double[EXIT_ANGLE_CANDIDATES.length], entryRadianBounds = new double[ENTRY_ANGLE_BOUNDS.length];
    private double optimalEntryRadian;
    private double dragCoef, magnusCoef;

    public BallisticModelSolver(LinearEquation velocityApproximator, double dragCoef, double magnusCoef) {
        super(velocityApproximator);
        this.dragCoef = dragCoef; this.magnusCoef = magnusCoef;
        // we need transient states to evaluate angle candidates
        // fill with empty states
        for (int i = 0; i < transientStates.length; i++) { transientStates[i] = new BallisticState(); }
    }

    public BallisticModelSolver(LinearEquation velocityApproximator, double dragCoef) { this(velocityApproximator, dragCoef, 0.0); }

    public BallisticModelSolver(LinearEquation velocityApproximator) { this(velocityApproximator, 0.0, 0.0); }

    /* GETTERS */

    public BallisticState state() { return state; }

    /* FUNCTIONAL METHODS */

    public void compute(double dx, double dy, double dz) {
        computeConversions();
        // "r" is the horizontal component in the direction of (dx, dy)--ie. the shooting plane
        // we assume the turret is locked on
        double r = Math.hypot(dx, dy);
        // all uses of a zero vector are static
        // hence we reuse the same object
        Vector zeroVector2d = Vector.zero(2);
        // instanciate
        BallisticState candState = new BallisticState();

        /* THETA LOOP */
        for (int i = 0; i < exitRadianCandidates.length; i++) {
            // run sim with this exit angle
            // velocity loop
            double exitAngle = exitRadianCandidates[i];

            // tangential velocity candidates
            // save error alongside for guess scaling
            // methods using past 2 samples are supported
            double[] endpoints = {0.0, 0.0};
            double[] epErrors = {0.0, 0.0};

            /* TAN VEL LOOP */
            for (int j = 0; true; j++) {
                // get scalar exit velocity from tangential velocity
                double exitVelocity = sampleCandidate(endpoints, epErrors, r, dz, exitAngle, j);
                // no need to convert tangential to exit---endpoints are sampled in exit space

                // sim parameters
                Vector p = Vector.zero(2); // position in the exit frame
                Vector v = new Vector(exitVelocity * Math.cos(exitAngle), exitVelocity * Math.sin(exitAngle));
                Vector a = new Vector(0.0, -G_IN); // begin acceleration without drag

                /* SIMULATION LOOP */
                double lastX = 0.0; double lastZ = 0.0;
                double slope, intercept, error; double estZ = 0.0;
                // check the sim value of [t_exit, v_tan]
                int k = 0;
                lastEndpoint = -1;
                error = dz; // true at t=0
                while (true) {
                    // initialize as zero vector
                    Vector drag = zeroVector2d; Vector magnus = zeroVector2d;

                    /* COMPUTE VELOCITY-DEPENDENT FORCES */

                    // compute drag magnitude
                    double mag = v.mag();
                    if (dragCoef != 0.0) drag = v.mul(dragCoef * mag);

                    if (magnusCoef != 0.0) {
                        // magnus generates perpendicular acceleration
                        Vector perpV = Vector.normal(v);
                        // reverse perpendicular direction to correct sign
                        if (Math.signum(perpV.at(1)) != APPLICATION_DIRECTION)
                            perpV.neg(); // in-place

                        // vector needs to be in the direction given by spin
                        // backspin = thrust; forward spin = drag
                        magnus = perpV.mul(magnusCoef * mag);
                    }

                    // update kinematic parameters
                    a.set(magnus.at(0) - drag.at(0), magnus.at(1) - G_IN - drag.at(1));
                    v.add(a.mul(DT)); // we edit a in-place to avoid allocating new objects---it's overridden prior anyways
                    p.add(Vector.mul(v, DT)); // we need to retain v, so we allocate a new Vector in the process

                    /* CHECK STOPPING CONDITION */
                    // by checking horizontal distance instead of vertical, we know the sim must converge
                    if (p.at(0) >= r) { // stop if beyond goal distance
                        // interpolate the true collision value
                        // z(x)
                        slope = (p.at(1) - lastZ) / (p.at(0) - lastX);
                        intercept = p.at(1) - slope * p.at(0);
                        // get estimated value (intercept with h=dz)
                        estZ = slope * r + intercept;

                        // update the correct endpoint
                        error = dz - estZ;
                        int targetIndex = error > 0.0 ? 0 : 1;
                        // apply the illinois trick
                        // if candidate velocities hit the same error-sign twice, halve the stuck side
                        // the OTHER side
                        if (targetIndex == lastEndpoint) epErrors[1-targetIndex] /= 2.0;
                        lastEndpoint = targetIndex;
                        // if negative
                        endpoints[targetIndex] = exitVelocity; epErrors[targetIndex] = error;
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
                if (j >= MAX_SOLVER_ITERATIONS) {
                    // return an invalid state
                    transientStates[i].updateInvalid();
                    break;
                }
            }
        }

        // accept the best state
        candState.mirror(transientStates[0]);
        // iteratively select/discard
        for (int j = 1; j < transientStates.length; j++){
            if (transientStates[j].valid() && valueInBounds(candState.entryTheta(), entryRadianBounds) && compareStates(candState, transientStates[j])) candState.mirror(transientStates[j]);
        }
        // now the user can pull "state" for shot prediction
        // arguments should already be passed with SwM applied
        state.update(candState);
    }

    /* HELPERS */

    private double sampleCandidate(double[] endpoints, double[] epErrors, double r, double dz, double exitRadians, int i) {
        // first two sim iterations---no basis
        // hence no use of epErrors
        if (i < 2) {
            // try to sample on past state
            if (state.valid()) {
                endpoints[0] = PAST_MIN_SCALER * state.exitVelocity(); endpoints[1] = PAST_MAX_SCALER * state.exitVelocity();
            }
            // then estimate based on closed-form solution
            else {
                double theoryMin = Math.sqrt((G_IN * r * r) / (2 * Math.pow(Math.cos(exitRadians), 2) * (r * Math.tan(exitRadians) - dz)));
                endpoints[0] = THEORETICAL_MIN_SCALER * theoryMin; endpoints[1] = THEORETICAL_MAX_SCALER * theoryMin;
            }
            // then return the correct value
            return endpoints[i];
        }

        // NOT first two iterations---can use saved history
        // the first branch returns, this doesn't need an else
        // else, make informed guesses by a root finder

        // expand solution range if no root between exists
        if (epErrors[0] * epErrors[1] > 0.0) endpoints[0] *= LOWER_RANGE_EXTENDER; endpoints[1] *= UPPER_RANGE_EXTENDER;

        return regulaFalsi(endpoints, epErrors);
    }

    private boolean compareStates(BallisticState curState, BallisticState canState) {
        // accept a lesser error if it's inside the angle bounds
        double diffError, diffAngle, diffSpeed;

        // establish a score weighing 1) proximity to the optimal entry angle
        // 2) minimizing error
        // 3) minimizing tangential speed changing
        diffError = curState.error() - canState.error();
        diffAngle = Math.abs(state.entryTheta() - curState.entryTheta()) - Math.abs(state.entryTheta() - canState.entryTheta());
        diffSpeed = Math.abs(state.exitVelocity() - curState.exitVelocity()) - Math.abs(state.exitVelocity() - canState.exitVelocity()); // current velocity cancels

        // if the score is positive, we say state2. if its negative, then state1
        double score = ERROR_WEIGHT * diffError + ANGLE_WEIGHT * diffAngle + SPEED_WEIGHT * diffSpeed;
        return score > 0.0;
    }

    private static double regulaFalsi(double[] endpoints, double[] epErrors) {
        // compute the root
        // since this is a linear APPROXIMATION, it won't be perfect
        // but it will ideally converge
        return endpoints[0] - epErrors[0] * (endpoints[1] - endpoints[0]) / (epErrors[1] - epErrors[0]);
    }

    @Override
    protected void computeConversions() {
        for (int i = 0; i < exitRadianCandidates.length; i++) exitRadianCandidates[i] = Math.toRadians(EXIT_ANGLE_CANDIDATES[i]);
        for (int i = 0; i < entryRadianBounds.length; i++) entryRadianBounds[i] = Math.toRadians(ENTRY_ANGLE_BOUNDS[i]);
        optimalEntryRadian = Math.toRadians(OPTIMAL_ENTRY_ANGLE);
    }

    private boolean valueInBounds(double value, double[] inclusiveBounds) { return inclusiveBounds[0] <= value || value <= inclusiveBounds[1]; }
}
