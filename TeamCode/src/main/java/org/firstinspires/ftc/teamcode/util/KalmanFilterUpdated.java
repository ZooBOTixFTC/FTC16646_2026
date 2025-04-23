package org.firstinspires.ftc.teamcode.util;

public class KalmanFilterUpdated {
    // State variables: position and velocity (x, y)
    private double stateX, stateY;
    private double velocityX, velocityY;

    // State uncertainties (error covariance)
    private double uncertaintyX, uncertaintyY;
    private double uncertaintyVX, uncertaintyVY;

    // Process noise (uncertainty in prediction step)
    private double processNoiseX, processNoiseY;
    private double processNoiseVX, processNoiseVY;

    // Measurement noise (uncertainty in Limelight readings)
    private double measurementNoiseX, measurementNoiseY;

    // Time step (adjust dynamically based on loop timing)
    private double lastUpdateTime;

    // Constructor: initialize state variables and noise values
    public KalmanFilterUpdated(double initialX, double initialY, double initialVX, double initialVY,
                        double initialUncertaintyX, double initialUncertaintyY,
                        double processNoiseX, double processNoiseY,
                        double processNoiseVX, double processNoiseVY,
                        double measurementNoiseX, double measurementNoiseY) {
        this.stateX = initialX;
        this.stateY = initialY;
        this.velocityX = initialVX;
        this.velocityY = initialVY;

        this.uncertaintyX = initialUncertaintyX;
        this.uncertaintyY = initialUncertaintyY;
        this.uncertaintyVX = processNoiseVX;
        this.uncertaintyVY = processNoiseVY;

        this.processNoiseX = processNoiseX;
        this.processNoiseY = processNoiseY;
        this.processNoiseVX = processNoiseVX;
        this.processNoiseVY = processNoiseVY;

        this.measurementNoiseX = measurementNoiseX;
        this.measurementNoiseY = measurementNoiseY;

        this.lastUpdateTime = System.nanoTime(); // Initialize time tracking
    }

    // Prediction step using encoder data (accounting for velocity)
    public void predict(double encoderX, double encoderY, double encoderVX, double encoderVY) {
        double currentTime = System.nanoTime();
        double dt = (currentTime - lastUpdateTime) / 1e9; // Convert ns to seconds
        lastUpdateTime = currentTime;

        // Predict position using velocity (linear motion assumption)
        stateX += velocityX * dt;
        stateY += velocityY * dt;

        // Update velocity with encoder readings
        velocityX = encoderVX;
        velocityY = encoderVY;

        // Increase uncertainties based on process noise and speed
        uncertaintyX += processNoiseX * Math.abs(velocityX);
        uncertaintyY += processNoiseY * Math.abs(velocityY);
        uncertaintyVX += processNoiseVX * Math.abs(encoderVX);
        uncertaintyVY += processNoiseVY * Math.abs(encoderVY);
    }

    // Correction step using Limelight data
    public void correct(double limelightX, double limelightY) {
        // Calculate Kalman gain for position
        double kalmanGainX = uncertaintyX / (uncertaintyX + measurementNoiseX);
        double kalmanGainY = uncertaintyY / (uncertaintyY + measurementNoiseY);

        // Update position estimate
        stateX = stateX + kalmanGainX * (limelightX - stateX);
        stateY = stateY + kalmanGainY * (limelightY - stateY);

        // Update uncertainty (reduce trust in prediction when Limelight gives data)
        uncertaintyX = (1 - kalmanGainX) * uncertaintyX;
        uncertaintyY = (1 - kalmanGainY) * uncertaintyY;

        // If Limelight has strong confidence, stabilize velocity
        if (kalmanGainX > 0.7) velocityX *= 0.5;
        if (kalmanGainY > 0.7) velocityY *= 0.5;
    }

    // Get current filtered state (position and velocity)
    public double[] getFilteredState() {
        return new double[]{stateX, stateY, velocityX, velocityY};
    }

    // Adjust measurement noise dynamically (e.g., trusting Limelight more with multiple tags)
    public void adjustMeasurementNoise(double noiseX, double noiseY) {
        measurementNoiseX = noiseX;
        measurementNoiseY = noiseY;
    }
}
