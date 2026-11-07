package org.firstinspires.ftc.teamcode.lm1.math.linalg;

import androidx.annotation.NonNull;

import java.util.Arrays;

public class Vector {
    private double[] values;
    private int dim;

    private enum Operation { ADD, SUBTRACT, MULTIPLY, DIVIDE, EXPONENTIATE }

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

    public static Matrix asMatrix(Vector v, Matrix dest) { return new Matrix(1, v.dim, v.values); }

    public static Matrix asMatrix(Vector v) { return asMatrix(v, Matrix.zero(1, v.dim)); }

    public Matrix asMatrix() { return asMatrix(this, Matrix.zero(1, dim)); }

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
    // in-place
    public Vector add(Vector v)                                  { return arithmetic(this, v, this, Operation.ADD); }
    // static
    public static Vector add(Vector v1, Vector v2)               { return arithmetic(v1, v2, zero(v1.dim), Operation.ADD); }
    // static with a destination
    public static Vector add(Vector v1, Vector v2, Vector dest)  { return arithmetic(v1, v2, dest, Operation.ADD); }

    // scalar addition
    public Vector add(double n)                                { return arithmetic(this, n, this, Operation.ADD); }
    public static Vector add(Vector v, double n)               { return arithmetic(v, n, zero(v.dim), Operation.ADD); }
    public static Vector add(Vector v, double n, Vector dest)  { return arithmetic(v, n, dest, Operation.ADD); }


    // vector subtraction
    public Vector sub(Vector v)                                  { return arithmetic(this, v, this, Operation.SUBTRACT); }
    public static Vector sub(Vector v1, Vector v2)               { return arithmetic(v1, v2, zero(v1.dim), Operation.SUBTRACT); }
    public static Vector sub(Vector v1, Vector v2, Vector dest)  { return arithmetic(v1, v2, dest, Operation.SUBTRACT); }

    // scalar addition
    public Vector sub(double n)                                { return arithmetic(this, n, this, Operation.SUBTRACT); }
    public static Vector sub(Vector v, double n)               { return arithmetic(v, n, zero(v.dim), Operation.SUBTRACT); }
    public static Vector sub(Vector v, double n, Vector dest)  { return arithmetic(v, n, dest, Operation.SUBTRACT); }


    // vector multiplication
    public Vector mul(Vector v)                                  { return arithmetic(this, v, this, Operation.MULTIPLY); }
    public static Vector mul(Vector v1, Vector v2)               { return arithmetic(v1, v2, zero(v1.dim), Operation.MULTIPLY); }
    public static Vector mul(Vector v1, Vector v2, Vector dest)  { return arithmetic(v1, v2, dest, Operation.MULTIPLY); }

    // scalar multiplication
    public Vector mul(double n)                                { return arithmetic(this, n, this, Operation.MULTIPLY); }
    public static Vector mul(Vector v, double n)               { return arithmetic(v, n, zero(v.dim), Operation.MULTIPLY); }
    public static Vector mul(Vector v, double n, Vector dest)  { return arithmetic(v, n, dest, Operation.MULTIPLY); }


    // vector division
    public Vector div(Vector v)                                  { return arithmetic(this, v, this, Operation.DIVIDE); }
    public static Vector div(Vector v1, Vector v2)               { return arithmetic(v1, v2, zero(v1.dim), Operation.DIVIDE); }
    public static Vector div(Vector v1, Vector v2, Vector dest)  { return arithmetic(v1, v2, dest, Operation.DIVIDE); }

    // scalar division
    public Vector div(double n)                                { return arithmetic(this, n, this, Operation.DIVIDE); }
    public static Vector div(Vector v, double n)               { return arithmetic(v, n, zero(v.dim), Operation.DIVIDE); }
    public static Vector div(Vector v, double n, Vector dest)  { return arithmetic(v, n, dest, Operation.DIVIDE); }


    // exponentiation
    public Vector pow(double n)                                { return arithmetic(this, n, this, Operation.EXPONENTIATE); }
    public static Vector pow(Vector v, double n)               { return arithmetic(v, n, zero(v.dim), Operation.EXPONENTIATE); }
    public static Vector pow(Vector v, double n, Vector dest)  { return arithmetic(v, n, dest, Operation.EXPONENTIATE); }

    // dot product
    public static double dot(Vector v1, Vector v2) {
        enforceSameDim(v1, v2);
        // alias
        double[] a = v1.values, b = v2.values;
        double dim = v1.dim, total = 0.0;
        for (int i = 0; i < dim; i++) total += a[i] * b[i];
        return total;
    }

    public double dot(Vector v) { return dot(this, v); }

    // negation

    public Vector neg()                                { return arithmetic(this, -1.0, this, Operation.MULTIPLY); }
    public static Vector neg(Vector v)               { return arithmetic(v, -1.0, zero(v.dim), Operation.MULTIPLY); }
    public static Vector neg(Vector v, Vector dest)  { return arithmetic(v, -1.0, dest, Operation.MULTIPLY); }

    // perpendicularity

    public static Vector normal(Vector v, Vector dest) {
        // only implemented for 2d vectors
        if (v.dim != 2) throw new UnsupportedOperationException("Vector.normal() is only implemented for 2D vectors");
        // alias
        double[] o = dest.values, a = v.values;
        // get temp (in case the same object is passed)
        double temp = a[0];
        o[0] = a[1]; o[1] = -temp;
        return dest;
    }

    public static Vector normal(Vector v) { return normal(v, zero(v.dim)); }

    public Vector normal() { return normal(this, this); }

    // miscellaneous functional methods

    public double distance(Vector v) { return sub(v).mag(); }
    public boolean withinDistance(Vector v, double threshold) { return distance(v) <= threshold; }

    /* FUNCTIONAL METHODS */

    public void from(Vector v) {
        enforceSameDim(this, v);
        System.arraycopy(v.values, 0, values, 0, values.length); dim = v.dim;
    }

    /* FACTORY */

    public static Vector zero(int dim) {
        double[] values = new double[dim];
        Arrays.fill(values, 0.0);
        return new Vector(values);
    }

    @NonNull
    public Vector clone() { return new Vector(values.clone()); }

    /* KERNELS */

    // vector-vector
    private static Vector arithmetic(Vector v1, Vector v2, Vector dest, Operation op) {
        enforceSameDim(v1, v2, dest);
        // alias
        double[] a = v1.values, b = v2.values, o = dest.values;
        double dim = v1.dim;
        // select the correct operation
        // selecting once and rewriting the loop adds redundancy but reduces overhead
        switch (op) {
            case ADD:      for (int i = 0; i < dim; i++) o[i] = a[i] + b[i];
            case SUBTRACT: for (int i = 0; i < dim; i++) o[i] = a[i] - b[i];
            case MULTIPLY: for (int i = 0; i < dim; i++) o[i] = a[i] * b[i];
            case DIVIDE:   for (int i = 0; i < dim; i++) o[i] = a[i] / b[i];
        }
        // 1) for logically returning the output nonexistent elsewhere
        // 2) for enabling chaining of vector ops
        return dest;
    }

    // vector-scalar
    private static Vector arithmetic(Vector v, double n, Vector dest, Operation op) {
        enforceSameDim(v, dest);
        // alias
        double[] a = v.values, o = dest.values;
        double dim = v.dim;
        // select the correct operation
        // selecting once and rewriting the loop adds redundancy but reduces overhead
        switch (op) {
            case ADD:          for (int i = 0; i < dim; i++) o[i] = a[i] + n; break;
            case SUBTRACT:     for (int i = 0; i < dim; i++) o[i] = a[i] - n; break;
            case MULTIPLY:     for (int i = 0; i < dim; i++) o[i] = a[i] * n; break;
            case DIVIDE:       for (int i = 0; i < dim; i++) o[i] = a[i] / n; break;
            case EXPONENTIATE: for (int i = 0; i < dim; i++) o[i] = Math.pow(a[i], n); break;
        }
        // 1) for logically returning the output nonexistent elsewhere
        // 2) for enabling chaining of vector ops
        return dest;
    }

    /* HELPERS */

    static void enforceSameDim(Vector... v) {
        int expDim = v[0].dim;
        // compare each vector to the first dimension
        for (int i = 1; i < v.length; i++) if (v[i].dim != expDim) throw new IllegalArgumentException("Vectors must have identical dimensions");
    }
}
