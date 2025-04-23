package org.firstinspires.ftc.teamcode.util;

public class KalmanFilter {
    private double stateX, stateY; // Position
    private double velocityX, velocityY; // Velocity

    private double uncertaintyX, uncertaintyY;
    private double uncertaintyVX, uncertaintyVY;

    private double processNoiseX, processNoiseY;
    private double processNoiseVX, processNoiseVY;

    private double measurementNoiseX, measurementNoiseY;

    private double lastUpdateTime; // Stores the last update time

    public KalmanFilter(double initialX, double initialY, double initialVX, double initialVY,
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

        this.lastUpdateTime = System.nanoTime();
    }

    public void predict(double encoderX, double encoderY, double encoderVX, double encoderVY) {
        double currentTime = System.nanoTime();
        double dt = (currentTime - lastUpdateTime) / 1e9; // Convert nanoseconds to seconds
        lastUpdateTime = currentTime;

        // Predict position using velocity over elapsed time
        stateX += velocityX * dt;
        stateY += velocityY * dt;

        // Update velocity
        velocityX = encoderVX;
        velocityY = encoderVY;

        // Increase uncertainty based on process noise and movement
        uncertaintyX += processNoiseX * Math.abs(velocityX) * dt;
        uncertaintyY += processNoiseY * Math.abs(velocityY) * dt;
        uncertaintyVX += processNoiseVX * dt;
        uncertaintyVY += processNoiseVY * dt;
    }

    public void correct(double limelightX, double limelightY) {
        double kalmanGainX = uncertaintyX / (uncertaintyX + measurementNoiseX);
        double kalmanGainY = uncertaintyY / (uncertaintyY + measurementNoiseY);

        // Update state position
        stateX += kalmanGainX * (limelightX - stateX);
        stateY += kalmanGainY * (limelightY - stateY);

        // Reduce uncertainty based on measurement confidence
        uncertaintyX *= (1 - kalmanGainX);
        uncertaintyY *= (1 - kalmanGainY);
    }

    public double[] getFilteredState() {
        return new double[]{stateX, stateY, velocityX, velocityY};
    }

    public void adjustMeasurementNoise(double noiseX, double noiseY) {
        measurementNoiseX = noiseX;
        measurementNoiseY = noiseY;
    }
}
