package org.firstinspires.ftc.teamcode.lm1.math.linalg;

public class Matrix {
    private double[] values;
    private int rows, cols;

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

    public Matrix add(Matrix mat) {
        // confirm dimensionality
        if (rows != mat.rows || cols != mat.cols) throw new IllegalArgumentException("Can not add dimensions of the given matrix");
        // assemble
        for (int i = 0; i < values.length; i++) values[i] = values[i] + mat.values[i];
        return this;
    }

    public static Matrix add(Matrix mat1, Matrix mat2) {
        // confirm dimensionality
        if (mat1.rows != mat2.rows || mat1.cols != mat2.cols) throw new IllegalArgumentException("Can not add dimensions of the given matrix");
        double[] newVals = new double[mat1.rows*mat1.cols];
        // assemble
        for (int i = 0; i < newVals.length; i++) newVals[i] = mat1.values[i] + mat2.values[i];
        return new Matrix(mat1.rows, mat1.cols, newVals);
    }

    public Matrix sub(Matrix mat) {
        // confirm dimensionality
        if (rows != mat.rows || cols != mat.cols) throw new IllegalArgumentException("Can not subtract dimensions of the given matrix");
        // assemble
        for (int i = 0; i < values.length; i++) values[i] = values[i] - mat.values[i];
        return this;
    }

    public static Matrix sub(Matrix mat1, Matrix mat2) {
        // confirm dimensionality
        if (mat1.rows != mat2.rows || mat1.cols != mat2.cols) throw new IllegalArgumentException("Can not subtract dimensions of the given matrix");
        double[] newVals = new double[mat1.rows*mat1.cols];
        // assemble
        for (int i = 0; i < newVals.length; i++) newVals[i] = mat1.values[i] - mat2.values[i];
        return new Matrix(mat1.rows, mat1.cols, newVals);
    }

    public Matrix scale(double n) {
        for (int i = 0; i < values.length; i++) values[i] *= n;
        return this;
    }

    public static Matrix scale(Matrix mat, double n) {
        double[] newVals = new double[mat.values.length];
        for (int i = 0; i < mat.values.length; i++) { mat.values[i] *= n; }
        return new Matrix(mat.rows, mat.cols, newVals);
    }

    public Matrix elemmul(Matrix mat) {
        // confirm dimensionality
        if (rows != mat.rows || cols != mat.cols) throw new IllegalArgumentException("Can not multiply dimensions of the given matrix");
        // assemble
        for (int i = 0; i < values.length; i++) values[i] = values[i] * mat.values[i];
        return this;
    }

    public static Matrix elemmul(Matrix mat1, Matrix mat2) {
        // confirm dimensionality
        if (mat1.rows != mat2.rows || mat1.cols != mat2.cols) throw new IllegalArgumentException("Can not multiply dimensions of the given matrix");
        double[] newVals = new double[mat1.rows*mat1.cols];
        // assemble
        for (int i = 0; i < newVals.length; i++) newVals[i] = mat1.values[i] * mat2.values[i];
        return new Matrix(mat1.rows, mat1.cols, newVals);
    }

    public Matrix div(Matrix mat) {
        // confirm dimensionality
        if (rows != mat.rows || cols != mat.cols) throw new IllegalArgumentException("Can not divide dimensions of the given matrix");
        // assemble
        for (int i = 0; i < values.length; i++) values[i] = values[i] / mat.values[i];
        return this;
    }

    public static Matrix div(Matrix mat1, Matrix mat2) {
        // confirm dimensionality
        if (mat1.rows != mat2.rows || mat1.cols != mat2.cols) throw new IllegalArgumentException("Can not divide dimensions of the given matrix");
        double[] newVals = new double[mat1.rows*mat1.cols];
        // assemble
        for (int i = 0; i < newVals.length; i++) newVals[i] = mat1.values[i] / mat2.values[i];
        return new Matrix(mat1.rows, mat1.cols, newVals);
    }

    public Matrix neg() {
        // assemble
        for (int i = 0; i < values.length; i++) values[i] = -values[i];
        return this;
    }

    public static Matrix neg(Matrix mat) {
        double[] newVals = new double[mat.rows*mat.cols];
        // assemble
        for (int i = 0; i < newVals.length; i++) newVals[i] = -mat.values[i];
        return new Matrix(mat.rows, mat.cols, newVals);
    }

    // computes this * vector
    public Vector mul(Vector v) {
        // confirm dimensionality
        if (v.dim() != cols) throw new IllegalArgumentException("Inner dimensions of the given vector and matrix do not align");
        double[] newVals = new double[rows];
        for (int i = 0; i < rows; i++) {
            double dot = 0.0;
            for (int j = 0; j < cols; j++) dot += v.at(j) * at(j, i);
            // set element in vector
            newVals[i] = dot;
        }
        // return an assembled vector
        return new Vector(newVals);
    }

    // computes this * mat
    public Matrix mul(Matrix mat) {
        // confirm dimensionality
        if (mat.rows != cols) throw new IllegalArgumentException("Inner dimensions of the given matrix and matrix do not align");
        double[] newVals = new double[rows*mat.cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < mat.cols; j++) {
                double dot = 0.0;
                for (int k = 0; k < cols; k++) dot += at(i, k) * mat.at(k, j);
                // set dot to correct insight
                set(newVals, dot, mat.cols, i, j);
            }
        }
        values = newVals;
        return this;
    }

    // computes this * mat
    public static Matrix mul(Matrix mat1, Matrix mat2) {
        // confirm dimensionality
        if (mat1.cols != mat2.rows) throw new IllegalArgumentException("Inner dimensions of the given matrix and matrix do not align");
        double[] newVals = new double[mat1.rows*mat2.cols];
        for (int i = 0; i < mat1.rows; i++) {
            for (int j = 0; j < mat2.cols; j++) {
                double dot = 0.0;
                for (int k = 0; k < mat1.cols; k++) dot += mat1.at(i, k) * mat2.at(k, j);
                // set dot to correct insight
                set(newVals, dot, mat2.cols, i, j);
            }
        }
        // return an assembled mat
        return new Matrix(mat1.rows, mat2.cols, newVals);
    }

    public Matrix transpose() {
        double[] newVals = new double[cols*rows];
        for (int i = 0; i < rows; i++) {
            // swap placing
            for (int j = 0; j < cols; j++) { set(newVals, at(i, j), rows, j, i); }
        }
        // push to values
        values = newVals;
        // swap dimensions
        int tempRow = rows;
        rows = cols; cols = tempRow;
        return this;
    }

    public static Matrix transpose(Matrix mat) {
        double[] newVals = new double[mat.cols*mat.rows];
        for (int i = 0; i < mat.rows; i++) {
            // swap placing
            for (int j = 0; j < mat.cols; j++) { set(newVals, mat.at(i, j), mat.rows, j, i); }
        }
        // return an assembled mat
        return new Matrix(mat.cols, mat.rows, newVals);
    }

    /* FACTORIES */

    public static Matrix identity(int size) {
        double[] values = new double[size*size];
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) { set(values, i==j ? 1.0 : 0.0, size, i, j); }
        }
        return new Matrix(size, size, values);
    }

    public Matrix clone() { return new Matrix(rows, cols, values.clone()); }

    /* HELPERS */

    private static double get(double[] flattenedArray, int cols, int row, int col) { return flattenedArray[col + cols*row]; }
    private static void set(double[] flattenedArray, double value, int cols, int row, int col) { flattenedArray[col + cols*row] = value; }
}
