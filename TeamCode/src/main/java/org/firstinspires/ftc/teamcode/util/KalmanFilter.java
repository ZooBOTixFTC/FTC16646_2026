package org.firstinspires.ftc.teamcode.util;

/**
 * Simple linear Kalman filter implementation.
 * Self-contained: includes basic matrix operations.
 */
public class KalmanFilter {
    private double[][] x; // state vector
    private double[][] P; // covariance
    private double[][] F; // state transition
    private double[][] Q; // process noise
    private double[][] H; // measurement map
    private double[][] R; // measurement noise

    public KalmanFilter(double[][] initialState,
                        double[][] initialCovariance,
                        double[][] stateTransition,
                        double[][] processNoise,
                        double[][] measurementMap,
                        double[][] measurementNoise) {
        this.x = initialState;
        this.P = initialCovariance;
        this.F = stateTransition;
        this.Q = processNoise;
        this.H = measurementMap;
        this.R = measurementNoise;
    }

    /** Predict step: propagate state forward */
    public void predict() {
        x = multiply(F, x);
        P = add(multiply(F, multiply(P, transpose(F))), Q);
    }

    /** Update step: incorporate measurement z */
    public void update(double[][] z) {
        double[][] y = subtract(z, multiply(H, x)); // innovation
        double[][] S = add(multiply(H, multiply(P, transpose(H))), R);
        double[][] K = multiply(P, multiply(transpose(H), invert(S)));

        // state update
        x = add(x, multiply(K, y));

        // covariance update
        double[][] I = identity(P.length);
        P = multiply(subtract(I, multiply(K, H)), P);
    }

    public double[][] getState() {
        return x;
    }

    // --- Matrix helper methods ---
    private double[][] multiply(double[][] A, double[][] B) {
        int rows = A.length;
        int cols = B[0].length;
        int inner = B.length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                for (int k = 0; k < inner; k++) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return result;
    }

    private double[][] add(double[][] A, double[][] B) {
        int rows = A.length;
        int cols = A[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result[i][j] = A[i][j] + B[i][j];
        return result;
    }

    private double[][] subtract(double[][] A, double[][] B) {
        int rows = A.length;
        int cols = A[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result[i][j] = A[i][j] - B[i][j];
        return result;
    }

    private double[][] transpose(double[][] A) {
        int rows = A.length;
        int cols = A[0].length;
        double[][] result = new double[cols][rows];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result[j][i] = A[i][j];
        return result;
    }

    private double[][] invert(double[][] A) {
        int n = A.length;
        double[][] aug = new double[n][2*n];
        // build augmented matrix [A | I]
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) aug[i][j] = A[i][j];
            aug[i][n+i] = 1.0;
        }
        // Gaussian elimination
        for (int i = 0; i < n; i++) {
            double pivot = aug[i][i];
            if (pivot == 0) throw new RuntimeException("Matrix not invertible");
            for (int j = 0; j < 2*n; j++) aug[i][j] /= pivot;
            for (int k = 0; k < n; k++) {
                if (k != i) {
                    double factor = aug[k][i];
                    for (int j = 0; j < 2*n; j++) {
                        aug[k][j] -= factor * aug[i][j];
                    }
                }
            }
        }
        // extract inverse
        double[][] inv = new double[n][n];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                inv[i][j] = aug[i][n+j];
        return inv;
    }

    private double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }
}
