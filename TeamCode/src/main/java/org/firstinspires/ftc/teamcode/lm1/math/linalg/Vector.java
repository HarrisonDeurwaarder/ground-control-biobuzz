package org.firstinspires.ftc.teamcode.lm1.math.linalg;

import androidx.annotation.NonNull;

import java.util.Arrays;

public class Vector {
    private double[] values;
    private int dim;

    public Vector(double... n) {
        values = new double[n.length]; // take the contents of the vector
        // dims = n elements
        dim = n.length;
    }

    /* GETTERS & SETTERS */

    public double[] values() { return values; }

    public int dim() { return dim; }

    public double at(int i) { return values[i]; }

    public void set(double... n) { System.arraycopy(values, 0, n, 0, dim); }

    /* OPERATIONS */

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

    public Vector norm() { return div(mag()); } // div operates in-place
    public static Vector norm(Vector v) { return v.div(v.mag()); }

    // vector addition
    public Vector add(Vector v) { // in-place
        enforceSameDim(this, v);
        // iteratively add
        for (int i = 0; i < dim; i++) values[i] = values[i] + v.values[i];
        return this;
    }

    public static Vector add(Vector v1, Vector v2) { // static
        enforceSameDim(v1, v2);
        // create an empty vector
        double[] values = new double[v1.dim];
        // iteratively add
        for (int i = 0; i < v1.dim; i++) values[i] = v1.values[i] + v2.values[i];
        return new Vector(values);
    }

    // scalar addition
    public Vector add(double n) { // in-place
        // iteratively add
        for (int i = 0; i < dim; i++) values[i] = values[i] + n;
        return this;
    }

    public static Vector add(Vector v, double n) { // static
        // create an empty vector
        double[] values = new double[v.dim];
        // iteratively add
        for (int i = 0; i < v.dim; i++) values[i] = v.values[i] + n;
        return new Vector(values);
    }

    // element-wise vector multiplication
    public Vector mul(Vector v) { // in-place
        enforceSameDim(this, v);
        // iteratively add
        for (int i = 0; i < dim; i++) values[i] = values[i] * v.values[i];
        return this;
    }

    public static Vector mul(Vector v1, Vector v2) { // static
        enforceSameDim(v1, v2);
        // create an empty vector
        double[] values = new double[v1.dim];
        // iteratively add
        for (int i = 0; i < v1.dim; i++) values[i] = v1.values[i] * v2.values[i];
        return new Vector(values);
    }

    // scalar multiplication
    public Vector mul(double n) { // in-place
        // iteratively add
        for (int i = 0; i < dim; i++) values[i] = values[i] * n;
        return this;
    }

    public static Vector mul(Vector v, double n) { // static
        // create an empty vector
        double[] values = new double[v.dim];
        // iteratively add
        for (int i = 0; i < v.dim; i++) values[i] = v.values[i] * n;
        return new Vector(values);
    }

    // scalar element-wise exponentiation
    public Vector pow(double n) { // in-place
        // iteratively add
        for (int i = 0; i < dim; i++) values[i] = Math.pow(values[i], n);
        return this;
    }

    public static Vector pow(Vector v, double n) { // static
        // create an empty vector
        double[] values = new double[v.dim];
        // iteratively add
        for (int i = 0; i < v.dim; i++) values[i] = Math.pow(v.values[i], n);
        return new Vector(values);
    }

    public double dot(Vector v) { // in-place
        enforceSameDim(this, v);
        double total = 0.0;
        // iteratively add
        // using .mul().sum() allocates a new object, this is faster
        for (int i = 0; i < dim; i++) { total += at(i) * v.at(i); }
        return total;
    }

    public static double dot(Vector v1, Vector v2) { // static
        enforceSameDim(v1, v2);
        double total = 0.0;
        // iteratively add
        // using .mul().sum() allocates a new object, this is faster
        for (int i = 0; i < v1.dim; i++) { total += v1.at(i) * v2.at(i); }
        return total;
    }

    // vector subtraction
    public Vector sub(Vector v) { // in-place
        enforceSameDim(this, v);
        // iteratively add
        for (int i = 0; i < dim; i++) values[i] = values[i] - v.values[i];
        return this;
    }

    public static Vector sub(Vector v1, Vector v2) { // static
        enforceSameDim(v1, v2);
        // create an empty vector
        double[] values = new double[v1.dim];
        // iteratively add
        for (int i = 0; i < v1.dim; i++) values[i] = v1.values[i] - v2.values[i];
        return new Vector(values);
    }

    // scalar subtraction
    public Vector sub(double n) { // in-place
        // iteratively add
        for (int i = 0; i < dim; i++) values[i] = values[i] - n;
        return this;
    }

    public static Vector sub(Vector v, double n) { // static
        // create an empty vector
        double[] values = new double[v.dim];
        // iteratively add
        for (int i = 0; i < v.dim; i++) values[i] = v.values[i] - n;
        return new Vector(values);
    }

    // element-wise vector division
    public Vector div(Vector v) { // in-place
        enforceSameDim(this, v);
        // iteratively add
        for (int i = 0; i < dim; i++) values[i] = values[i] / v.values[i];
        return this;
    }

    public static Vector div(Vector v1, Vector v2) { // static
        enforceSameDim(v1, v2);
        // create an empty vector
        double[] values = new double[v1.dim];
        // iteratively add
        for (int i = 0; i < v1.dim; i++) values[i] = v1.values[i] / v2.values[i];
        return new Vector(values);
    }

    // scalar multiplication
    public Vector div(double n) { // in-place
        // iteratively add
        for (int i = 0; i < dim; i++) values[i] = values[i] / n;
        return this;
    }

    public static Vector div(Vector v, double n) { // static
        // create an empty vector
        double[] values = new double[v.dim];
        // iteratively add
        for (int i = 0; i < v.dim; i++) values[i] = v.values[i] / n;
        return new Vector(values);
    }

    public Vector neg() { return mul(-1.0); } // in-place

    public static Vector neg(Vector v) { return mul(v, -1.0); } // static

    public Vector normal() {
        // only implemented for 2d vectors
        if (dim != 2) throw new UnsupportedOperationException("Vector.normal() is only implemented for 2D vectors");
        double temp = values[0];
        values[0] = values[1]; values[1] = temp;
        return this;
    }

    public static Vector normal(Vector v) {
        // only implemented for 2d vectors
        if (v.dim != 2) throw new UnsupportedOperationException("Vector.normal() is only implemented for 2D vectors");
        double[] values = new double[v.dim];
        values[0] = v.values[1]; values[1] = -v.values[0];
        return new Vector(values);
    }

    public double distance(Vector v) { return sub(v).mag(); }
    public boolean withinDistance(Vector v, double threshold) { return distance(v) <= threshold; }

    /* FACTORY */

    public static Vector zero(int dim) {
        double[] values = new double[dim];
        Arrays.fill(values, 0.0);
        return new Vector(values);
    }

    @NonNull
    public Vector clone() { return new Vector(values.clone()); }

    /* HELPERS */

    private static void enforceSameDim(Vector v1, Vector v2) {
        if (v1.dim() != v2.dim()) throw new IllegalArgumentException("'v1' and 'v2' must have identical dimensions");
    }
}
