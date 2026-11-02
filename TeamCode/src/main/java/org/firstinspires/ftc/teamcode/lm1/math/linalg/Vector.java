package org.firstinspires.ftc.teamcode.lm1.math.linalg;

import java.util.Arrays;

public class Vector {
    private final double[] values;
    private final int dim;

    private Vector(double... n) {
        values = n.clone(); // take the contents of the vector
        // dims = n elements
        dim = n.length;
    }

    /* GETTERS */

    public double[] values() { return values; }
    public int dim() { return dim; }
    public double at(int i) { return values[i]; }

    /* OPERATIONS */
    // vector is immutable so all operations return a new vector

    public double sum() {
        double total = 0;
        for (double n : values) total += n;
        return total;
    }

    public double mag() {
        double total = 0;
        for (double n : values) total += Math.pow(n, 2.0);
        return Math.sqrt(total);
    }

    public Vector norm() { return div(mag()); }

    // vector addition
    public Vector add(Vector v) {
        enforceSameDim(this, v);
        // create an empty vector
        double[] newVals = new double[dim];
        // iteratively add
        for (int i = 0; i < dim; i++) newVals[i] = values[i] + v.values[i];
        return vector(newVals);
    }

    // scalar addition
    public Vector add(double n) {
        // create an empty vector
        double[] newVals = new double[dim];
        // iteratively add
        for (int i = 0; i < dim; i++) newVals[i] = values[i] + n;
        return vector(newVals);
    }


    // element-wise vector multiplication
    public Vector mul(Vector v) {
        enforceSameDim(this, v);
        // create an empty vector
        double[] newVals = new double[dim];
        // iteratively add
        for (int i = 0; i < dim; i++) newVals[i] = values[i] * v.values[i];
        return vector(newVals);
    }

    // scalar multiplication
    public Vector mul(double n) {
        // create an empty vector
        double[] newVals = new double[dim];
        // iteratively add
        for (int i = 0; i < dim; i++) newVals[i] = values[i] * n;
        return vector(newVals);
    }

    // scalar element-wise exponentiation
    public Vector pow(double n) {
        // create an empty vector
        double[] newVals = new double[dim];
        // iteratively add
        for (int i = 0; i < dim; i++) newVals[i] = Math.pow(values[i], n);
        return vector(newVals);
    }


    public double dot(Vector v) { return mul(v).sum(); }


    public Vector sub(Vector v) { return add(v.flip()); }
    public Vector sub(double n) { return add(-n); }


    public Vector div(Vector v) { return mul(v.pow(-1.0)); }
    public Vector div(double n) { return mul(1.0 / n); }

    public Vector flip() { return mul(-1.0); }

    public double distance(Vector v) { return sub(v).mag(); }
    public boolean withinDistance(Vector v, double threshold) { return distance(v) <= threshold; }

    /* FACTORY */

    public static Vector vector(double... n) { return new Vector(n); }

    public static Vector zero(int dim) {
        double[] vals = new double[dim];
        Arrays.fill(vals, 0.0);
        return vector(vals);
    }

    /* HELPERS */

    private static void enforceSameDim(Vector v1, Vector v2) {
        if (v1.dim() != v2.dim()) throw new IllegalArgumentException("'v1' and 'v2' must have identical dimensions");
    }
}
