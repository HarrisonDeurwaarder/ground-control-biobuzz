package org.firstinspires.ftc.teamcode.lm1.math.linalg;

import androidx.annotation.NonNull;

import java.util.Arrays;

public class Matrix {
    private double[] values;
    private int rows, cols;
    private enum Operation { ADD, SUBTRACT, MULTIPLY, DIVIDE, EXPONENTIATE }

    public Matrix(double[]... values) {
        // confirm present arguments
        if (values.length == 0) throw new IllegalArgumentException("Can not create an empty matrix");
        // confirm homogenous dimensions
        int nomDim = values[0].length;
        for (int i = 1; i < values.length; i++) if (values[i].length != nomDim) throw new IllegalArgumentException("Matrix must have homogenous dimensions");
        // flatten values
        this.values = new double[values.length * nomDim];
        for (int i = 0; i < values.length; i++) {
            for (int j = 0; j < nomDim; j++) { this.values[j + nomDim*i] = values[i][j]; }
        }
        // save dimensions
        rows = values.length; cols = nomDim;
    }

    public Matrix(int rows, int cols, double... values) {
        // constructor is already in 1D form
        this.rows = rows; this.cols = cols;
        // check divisibility
        if (rows*cols != values.length) throw new IllegalArgumentException("Values can not be unflattened to specified dimensions");
        this.values = values.clone();
    }

    /* GETTERS */

    public double[][] values() {
        double[][] array2d = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) array2d[i][j] = Matrix.get(values, cols, i, j);
        }
        return array2d;
    }

    public double at(int row, int col) { return Matrix.get(values, cols, row, col); }
    public int rows() { return rows; }
    public int cols() { return cols; }

    /* OPERATIONS */

    // matrix addition
    // in-place
    public Matrix add(Matrix mat)                                   { return arithmetic(this, mat, this, Operation.ADD); }
    // static
    public static Matrix add(Matrix mat1, Matrix mat2)              { return arithmetic(mat1, mat2, zero(mat1.rows, mat1.cols), Operation.ADD); }
    // static with a destination
    public static Matrix add(Matrix mat1, Matrix mat2, Matrix dest) { return arithmetic(mat1, mat2, dest, Operation.ADD); }

    // scalar addition
    public Matrix add(double n)                                  { return arithmetic(this, n, this, Operation.ADD); }
    public static Matrix add(Matrix mat1, double n)              { return arithmetic(mat1, n, zero(mat1.rows, mat1.cols), Operation.ADD); }
    public static Matrix add(Matrix mat1, double n, Matrix dest) { return arithmetic(mat1, n, dest, Operation.ADD); }


    // matrix subtraction
    public Matrix sub(Matrix mat)                                   { return arithmetic(this, mat, this, Operation.SUBTRACT); }
    public static Matrix sub(Matrix mat1, Matrix mat2)              { return arithmetic(mat1, mat2, zero(mat1.rows, mat1.cols), Operation.SUBTRACT); }
    public static Matrix sub(Matrix mat1, Matrix mat2, Matrix dest) { return arithmetic(mat1, mat2, dest, Operation.SUBTRACT); }

    // scalar subtraction
    public Matrix sub(double n)                                  { return arithmetic(this, n, this, Operation.SUBTRACT); }
    public static Matrix sub(Matrix mat1, double n)              { return arithmetic(mat1, n, zero(mat1.rows, mat1.cols), Operation.SUBTRACT); }
    public static Matrix sub(Matrix mat1, double n, Matrix dest) { return arithmetic(mat1, n, dest, Operation.SUBTRACT); }


    // matrix multiplication
    public Matrix elemmul(Matrix mat)                                   { return arithmetic(this, mat, this, Operation.MULTIPLY); }
    public static Matrix elemmul(Matrix mat1, Matrix mat2)              { return arithmetic(mat1, mat2, zero(mat1.rows, mat1.cols), Operation.MULTIPLY); }
    public static Matrix elemmul(Matrix mat1, Matrix mat2, Matrix dest) { return arithmetic(mat1, mat2, dest, Operation.MULTIPLY); }

    // scalar multiplication
    public Matrix mul(double n)                                  { return arithmetic(this, n, this, Operation.MULTIPLY); }
    public static Matrix mul(Matrix mat1, double n)              { return arithmetic(mat1, n, zero(mat1.rows, mat1.cols), Operation.MULTIPLY); }
    public static Matrix mul(Matrix mat1, double n, Matrix dest) { return arithmetic(mat1, n, dest, Operation.MULTIPLY); }


    // matrix division
    public Matrix elemdiv(Matrix mat)                                   { return arithmetic(this, mat, this, Operation.DIVIDE); }
    public static Matrix elemdiv(Matrix mat1, Matrix mat2)              { return arithmetic(mat1, mat2, zero(mat1.rows, mat1.cols), Operation.DIVIDE); }
    public static Matrix elemdiv(Matrix mat1, Matrix mat2, Matrix dest) { return arithmetic(mat1, mat2, dest, Operation.DIVIDE); }

    // scalar division
    public Matrix div(double n)                                  { return arithmetic(this, n, this, Operation.DIVIDE); }
    public static Matrix div(Matrix mat1, double n)              { return arithmetic(mat1, n, zero(mat1.rows, mat1.cols), Operation.DIVIDE); }
    public static Matrix div(Matrix mat1, double n, Matrix dest) { return arithmetic(mat1, n, dest, Operation.DIVIDE); }

    // exponentiation
    // scalar division
    public Matrix pow(double n)                                  { return arithmetic(this, n, this, Operation.EXPONENTIATE); }
    public static Matrix pow(Matrix mat1, double n)              { return arithmetic(mat1, n, zero(mat1.rows, mat1.cols), Operation.EXPONENTIATE); }
    public static Matrix pow(Matrix mat1, double n, Matrix dest) { return arithmetic(mat1, n, dest, Operation.EXPONENTIATE); }

    // this * matrix
    public static Matrix mul(Matrix mat1, Matrix mat2, Matrix dest) {
        // checks
        enforceInnerDim(mat1, mat2);
        enforceCustomDim(dest, mat1.rows, mat2.cols);
        // first dim
        for (int i = 0; i < mat1.rows; i++) {
            // second dim
            for (int j = 0; j < mat2.cols; j++) {
                // dot over shared inner dim
                double total = 0.0;
                for (int k = 0; k < mat1.cols; k++) total += mat1.at(i, k) * mat2.at(k, j);
                // plant in dest
                set(dest.values, total, mat2.cols, i, j);
            }
        }
        return dest;
    }
    public static Matrix mul(Matrix mat1, Matrix mat2) { return mul(mat1, mat2, zero(mat1.rows, mat2.cols)); }
    public Matrix mul(Matrix mat) { return mul(this, mat, zero(rows, mat.cols)); }
    // no in-place call---can not modify shape after instantiation


    // this * vector
    public static Vector mul(Matrix mat, Vector v, Vector dest) {
        // checks
        enforceCustomDim(mat.cols, v.dim());
        enforceCustomDim(mat.rows, dest.dim());
        double[] a = v.values(), o = dest.values();
        // matrix dim
        // treat the vector as a column vect
        for (int i = 0; i < mat.rows; i++) {
            // operative dim
            double total = 0.0;
            for (int j = 0; j < mat.cols; j++) total += mat.at(i, j) * a[j];
            // plant in dest
            o[i] = total;
        }
        // treat the return as a row vector---idk
        return dest;
    }
    public static Vector mul(Matrix mat, Vector v) { return mul(mat, v, Vector.zero(mat.rows)); }
    public Vector mul(Vector v) { return mul(this, v, Vector.zero(rows)); }


    public static Matrix transpose(Matrix mat, Matrix dest) {
        enforceSwappedDim(mat, dest);
        double[] values = dest.values;
        for (int i = 0; i < mat.rows; i++) {
            // swap placing
            for (int j = 0; j < mat.cols; j++) { set(values, mat.at(i, j), mat.rows, j, i); }
        }
        // return an assembled mat
        return dest;
    }
    public static Matrix transpose(Matrix mat) { return transpose(mat, zero(mat.rows, mat.cols)); }
    // no in-place call---can not modify shape after instantiation

    /* FUNCTIONAL METHODS */

    public void from(Matrix mat) {
        enforceSameDim(this, mat);
        System.arraycopy(mat.values, 0, values, 0, values.length); rows = mat.rows; cols = mat.cols;
    }

    /* FACTORIES */

    public static Matrix identity(int size) {
        double[] values = new double[size*size];
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) { set(values, i==j ? 1.0 : 0.0, size, i, j); }
        }
        return new Matrix(size, size, values);
    }

    public static Matrix zero(int rows, int cols) {
        double[] values = new double[rows*cols];
        Arrays.fill(values, 0.0);
        return new Matrix(values);
    }

    @NonNull
    public Matrix clone() { return new Matrix(rows, cols, values.clone()); }

    /* HELPERS */

    // matrix-matrix
    private static Matrix arithmetic(Matrix mat1, Matrix mat2, Matrix dest, Operation op) {
        enforceSameDim(mat1, mat2, dest);
        // alias
        double[] a = mat1.values, b = mat2.values, o = dest.values;
        double len = a.length;
        // select the correct operation
        // selecting once and rewriting the loop adds redundancy but reduces overhead
        switch (op) {
            case ADD:      for (int i = 0; i < len; i++) o[i] = a[i] + b[i];
            case SUBTRACT: for (int i = 0; i < len; i++) o[i] = a[i] - b[i];
            case MULTIPLY: for (int i = 0; i < len; i++) o[i] = a[i] * b[i];
            case DIVIDE:   for (int i = 0; i < len; i++) o[i] = a[i] / b[i];
        }
        // 1) for logically returning the output nonexistent elsewhere
        // 2) for enabling chaining of vector ops
        return dest;
    }

    // matrix-scalar
    private static Matrix arithmetic(Matrix mat, double n, Matrix dest, Operation op) {
        enforceSameDim(mat, dest);
        // alias
        double[] a = mat.values, o = dest.values;
        double len = a.length;
        // select the correct operation
        // selecting once and rewriting the loop adds redundancy but reduces overhead
        switch (op) {
            case ADD:          for (int i = 0; i < len; i++) o[i] = a[i] + n; break;
            case SUBTRACT:     for (int i = 0; i < len; i++) o[i] = a[i] - n; break;
            case MULTIPLY:     for (int i = 0; i < len; i++) o[i] = a[i] * n; break;
            case DIVIDE:       for (int i = 0; i < len; i++) o[i] = a[i] / n; break;
            case EXPONENTIATE: for (int i = 0; i < len; i++) o[i] = Math.pow(a[i], n); break;
        }
        // 1) for logically returning the output nonexistent elsewhere
        // 2) for enabling chaining of matrix ops
        return dest;
    }

    private static double get(double[] flattenedArray, int cols, int row, int col) { return flattenedArray[col + cols*row]; }
    private static void set(double[] flattenedArray, double value, int cols, int row, int col) { flattenedArray[col + cols*row] = value; }

    static void enforceSameDim(Matrix... mat) {
        int expRows = mat[0].rows, expCols = mat[0].cols;
        // compare each matrix to the first dimensions
        for (int i = 1; i < mat.length; i++) if (mat[i].rows != expRows || mat[i].cols != expCols) throw new IllegalArgumentException("Matrices must have identical dimensions");
    }

    static void enforceSwappedDim(Matrix mat1, Matrix mat2) {
        // compare other matrix to swaps dimensions (for transpose)
        if (mat1.rows != mat2.cols || mat1.cols != mat2.rows) throw new IllegalArgumentException("Matrices must have swapped dimensions");
    }

    static void enforceInnerDim(Matrix mat1, Matrix mat2) {
        // compare only inner dims (for matmul)
        if (mat1.cols != mat2.rows) throw new IllegalArgumentException("Matrices must have identical inner dimensions");
    }

    static void enforceCustomDim(Matrix mat, int rows, int cols) {
        // compare matrix against provided dims
        if (mat.rows != rows || mat.cols != cols) throw new IllegalArgumentException("Matrix must match expected dimensions");
    }

    static void enforceCustomDim(int d1, int d2) {
        // compare provided dims
        if (d1 != d2) throw new IllegalArgumentException("Matrix must match expected dimensions");
    }
}
