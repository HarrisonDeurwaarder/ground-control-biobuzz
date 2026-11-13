package org.firstinspires.ftc.teamcode.lm1.math.diffeq.sampler;

public class RegulaFalsi implements Sampler {
    public static double GROWTH = 1.6;
    public static int REFINING_MAX_ITER = 100;

    private double lo, hi, loE, hiE, curSample;
    private double lo0, hi0, width, min, max;
    private Target lastTarget, target; private State state;
    private boolean needsError;
    private int bi, ri;

    private enum Target { NONE, LO, HI }
    private enum State { FILL, BRACKET, REFINE, FAIL }

    public RegulaFalsi(double lo0, double hi0, double min, double max) { reset(lo0, hi0, min, max); }

    /* GETTERS & SETTERS */

    public double lo() { return lo; }
    public double hi() { return hi; }
    public double loE() { return loE; }
    public double hiE() { return hiE; }

    public void logError(double mostRecentSampleError) {
        if (state == State.FAIL) return;

        // throw on calling logError twice per sample
        if (!needsError) throw new IllegalStateException("Can not call logError() more than once per sample");
        // i only get state based on error IF im in refinement
        // else, I already know which endpoint to update
        if (state == State.REFINE) { target = (Math.signum(loE) == Math.signum(mostRecentSampleError)) ? Target.LO : Target.HI; }

        if (target == Target.LO) { loE = mostRecentSampleError; lo = curSample; }
        else if (target == Target.HI) { hiE = mostRecentSampleError; hi = curSample; }
        else throw new IllegalStateException("Can not set error before sampling a value");

        needsError = false;
    }

    public boolean failed() { return state == State.FAIL; }

    /* FUNCTIONAL METHODS */

    public double sample() {
        if (state != State.FAIL) {
            raiseOnUnloggedError();
            needsError = true;
        }

        /* FILL */
        // use predetermined lo0 and hi0

        if (state == State.FILL) {
            // try to return lo
            if (Double.isNaN(lo)) {
                lo = lo0; target = Target.LO; curSample = lo;
            }
            // then try to return hi
            else if (Double.isNaN(hi)) {
                hi = hi0; target = Target.HI; curSample = hi;
            }
            // if both are filled, advance to the next state
            else state = State.BRACKET;
        }

        /* BRACKET */
        // expand predetermined lo0 and hi0 until they bound the root

        if (state == State.BRACKET) {
            double canLo = lo, canHi = hi;

            // bracket by steps
            // same signs means no root between
            if (loE * hiE > 0.0) {
                // exponentially increase width
                width *= GROWTH;
                // expand the side with error closer to zero
                if (Math.abs(loE) < Math.abs(hiE)) { canLo -= width; target = Target.LO; }
                else { canHi += width; target = Target.HI; }
            }

            // if they bound a root, advance
            // REFINE is the first to care about lastTarget, so reset it
            else { state = State.REFINE; target = Target.NONE; lastTarget = Target.NONE; }

            /* CHECK FAIL */
            boolean overbounds = (target == Target.LO) ? (canLo < min) : (canHi > max);
            if (state == State.BRACKET && overbounds) { state = State.FAIL; }
            // then update true lo/hi
            // and only update count on non-fail
            else { lo = canLo; hi = canHi; bi++; }
            // update curSample accordingly
            curSample = (target == Target.LO) ? lo : hi;
        }

        /* REFINE */
        // apply regula falsi

        if (state == State.REFINE) {
            // APPLY ILLINOIS
            // this is why we don't update lastTarget immediately---we must use it for this
            // if the last two updates were on the endpoint (i.e. the solver is stuck), pull it the other way
            if (target == lastTarget && target != Target.NONE) {
                if (target == Target.LO) {
                    hiE /= 2.0;
                } else loE /= 2.0;
            }
            lastTarget = target;

            // COMPUTE USING REGULA FALSI
            curSample = regulaFalsi();

            /* CHECK FAIL */
            if (ri > REFINING_MAX_ITER) { state = State.FAIL; }
            else ri++;
        }

        // return optimal of the two on FAIL
        if (state == State.FAIL) {
            return (Math.abs(loE) < Math.abs(hiE)) ? lo : hi;
        }

        lastTarget = target;
        return curSample;
    }

    public void reset(double lo0, double hi0, double min, double max) {
        // reset all attributes for a new solution
        lo = Double.NaN; hi = Double.NaN; loE = Double.NaN; hiE = Double.NaN; curSample = Double.NaN;
        needsError = false;
        this.lo0 = lo0; this.hi0 = hi0; width = hi0 - lo0; this.min = min; this.max = max;
        lastTarget = Target.NONE; target = Target.NONE; state = State.FILL;
        bi = 0; ri = 0;
    }

    public void reset(double lo0, double hi0) { reset(lo0, hi0, min, max); }

    public void reset() { reset(lo0, hi0, min, max); }

    public double regulaFalsi() {
        // condensed version
        // expressing find slope -> find intercept -> find root
        return hi - hiE * (hi - lo) / (hiE - loE);
    }

    /* HELPERS */

    private void raiseOnUnloggedError() { if (needsError) throw new IllegalStateException("Error must be set for previous sample before sampling a new candidate"); }
}
