package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.SimpleMechanism;

public class LinearMotion implements SimpleMechanism, Subsystem {
    public final String name;

    private final DcMotorEx[] motors;
    private final boolean[] motorsReversed;
    private final DcMotorEx encoder;
    private final boolean encoderReversed;
    private final double maximumSpeed;
    private double kp, ki, kd, kf;
    private double setPoint;

    private double previousError = 0;
    private double  integralSum = 0;
    private double targetVelocity;

    private boolean unjamming;

    public LinearMotion(
            String name,
            DcMotorEx[] motors, boolean[] motorsReversed,
            DcMotorEx encoder, boolean encoderReversed,
            double maximumSpeed,
            double kP,
            double kI,
            double kD,
            double kF
    ){
        this.name = name;

        this.motors = motors;
        this.motorsReversed = motorsReversed;
        this.encoder = encoder;
        this.encoderReversed = encoderReversed;
        this.maximumSpeed = maximumSpeed;
        this.kp = kP;
        this.ki = kI;
        this.kd = kD;
        this.kf = kF;

        for (DcMotor motor:motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        encoder.setDirection(DcMotorSimple.Direction.FORWARD);

        this.targetVelocity = 0;  //initialize
    }

    //(-1.0 to 1.0)
    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    // Get current velocity from the encoder

    public double getCurrentVelocity() {
        double rawVelocity = encoder.getVelocity();
        double normalizedVelocity = rawVelocity / maximumSpeed;
        return encoderReversed ? -normalizedVelocity : normalizedVelocity;
    }

    public void resetController() {
        this.integralSum = 0;
        this.previousError = 0;
        this.targetVelocity = 0;
    }

    public void periodic(){
        double currentVelocity = getCurrentVelocity();

        double error = targetVelocity - currentVelocity;

        //proportional term
        double pTerm = error * kp;

        // Integral Term
        integralSum += error * (1.0 / Constants.SystemConstants.kSysUpdateRateHz);
        double iTerm = integralSum * ki;

        // D Term
        // 4. Derivative Term
        double derivative = (error - previousError) / (1.0 / Constants.SystemConstants.kSysUpdateRateHz);
        double dTerm = derivative * kd;
        previousError = error; // update

        double fTerm = targetVelocity * kf;

        double totalPower = pTerm + iTerm + dTerm + fTerm;

        totalPower = Math.min(Math.max(totalPower, -1.0), 1.0);

        if(!unjamming) runPower(totalPower);
    }

    public void runPower(double power){
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower((power) * (motorsReversed[i] ? -1:1));
        }
    }

    @Override
    public void goToPosition(double setPoint) {
        this.setPoint = setPoint;
    }

    public double getCurrentSetPoint() {
        return this.setPoint;
    }

    public void setMotorsStop(){
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        unjamming = false;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocityRaw() {
        return encoder.getVelocity(); // original RPM
    }

    public double getVelocityError() {
        return targetVelocity - getCurrentVelocity();
    }

    public double getOutputPower() {
        return motors[0].getPower();
    }

    public double getPosition() {
        return encoder.getCurrentPosition();
    }

    public void setPIDF(double p, double i, double d, double f){
        this.kp = p;
        this.ki = i;
        this.kd = d;
        this.kf = f;
    }

    public void unjam(){
        unjamming = true;
        runPower(-1);
    }
}