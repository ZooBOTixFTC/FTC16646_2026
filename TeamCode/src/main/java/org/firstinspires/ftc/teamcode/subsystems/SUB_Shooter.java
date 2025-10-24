package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

@Config
public class SUB_Shooter extends SubsystemBase {
    private final OpMode m_opMode;
    private final DcMotorEx m_shooterMotorLeft;
    private final DcMotorEx m_shooterMotorRight;
    private final Servo m_kickerRight;
    private final Servo m_kickerLeft;
    private final VoltageSensor m_voltageSensor;
    private double m_targetVelLeft;
    private double m_targetVelRight;

    public static double leftP, leftV, leftS, leftVel, rightP, rightV, rightS, rightVel;

    private int lastPos = 0;
    private long lastTime = System.nanoTime();
    private double smoothedVelocity = 0;

    public SUB_Shooter(OpMode p_opMode) {
        m_opMode = p_opMode;
        m_shooterMotorLeft = m_opMode.hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        m_shooterMotorRight = m_opMode.hardwareMap.get(DcMotorEx.class, "shooterMotorRight");

        m_kickerLeft = m_opMode.hardwareMap.get(Servo.class, "kickerLeft");
        m_kickerRight = m_opMode.hardwareMap.get(Servo.class, "kickerRight");

        m_voltageSensor = m_opMode.hardwareMap.voltageSensor.iterator().next();

        m_kickerLeft.setDirection(Servo.Direction.FORWARD);
        m_kickerRight.setDirection(Servo.Direction.REVERSE);

        m_shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_shooterMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        m_shooterMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setTargetVelLeft(double velocity) {
        m_targetVelLeft = velocity;
    }

    public void setTargetVelRight(double velocity) {
        m_targetVelRight = velocity;
    }

    public double getVelocityLeft() {
        long currentTime = System.nanoTime();
        double deltaTimeSec = (currentTime - lastTime) / 1e9; // convert nanoseconds to milliseconds

        if (deltaTimeSec >= 0.02) { // 20ms loop
            int currentPos = m_shooterMotorLeft.getCurrentPosition();
            int deltaTicks = currentPos - lastPos;

            double ticksPerRev = 28.0;
            double revs = deltaTicks / ticksPerRev;
            double rawVelocity = (revs * 360.0) / deltaTimeSec;

            smoothedVelocity = ShooterConstants.kAlpha * rawVelocity + (1 - ShooterConstants.kAlpha) * smoothedVelocity;

            lastPos = currentPos;
            lastTime = currentTime;
        }

        return Math.round(smoothedVelocity);
    }

    public double getVelocityRight() {
        long currentTime = System.nanoTime();
        double deltaTimeSec = (currentTime - lastTime) / 1e9; // convert nanoseconds to milliseconds

        if (deltaTimeSec >= 0.02) { // 20ms loop
            int currentPos = m_shooterMotorRight.getCurrentPosition();
            int deltaTicks = currentPos - lastPos;

            double ticksPerRev = 28.0;
            double revs = deltaTicks / ticksPerRev;
            double rawVelocity = (revs * 360.0) / deltaTimeSec;

            smoothedVelocity = ShooterConstants.kAlpha * rawVelocity + (1 - ShooterConstants.kAlpha) * smoothedVelocity;

            lastPos = currentPos;
            lastTime = currentTime;
        }

        return Math.round(smoothedVelocity);
    }

    public double getTargetVelLeft(){
        return m_targetVelLeft;
    }

    public double getTargetVelRight(){
        return m_targetVelRight;
    }

    public void setKickPosLeft(double angDeg) {
        m_kickerLeft.setPosition(angDeg / 300);
    }

    public void setKickPosRight(double angDeg) {
        m_kickerRight.setPosition(angDeg / 300);
    }

    @Override
    public void periodic() {
        m_opMode.telemetry.addData("Shooter Vel Left", getVelocityLeft());
        m_opMode.telemetry.addData("Shooter Target Vel Left", getTargetVelLeft());
        m_opMode.telemetry.addData("Shooter Vel Right", getVelocityRight());
        m_opMode.telemetry.addData("Shooter Target Vel Right", getTargetVelRight());

        double leftVolts;
        double rightVolts;

        if(ShooterConstants.kTuningMode){
            setTargetVelLeft(leftVel);
            setTargetVelRight(rightVel);

            leftVolts =
                leftS +
                leftV * getTargetVelLeft() +
                leftP * getTargetVelLeft() - getVelocityLeft();

            rightVolts =
                rightS +
                rightV * getTargetVelRight() +
                rightP * getTargetVelRight() - getVelocityRight();
        }else{
            leftVolts =
                ShooterConstants.kSLeft +
                ShooterConstants.kVLeft * getTargetVelLeft() +
                ShooterConstants.kPLeft * getTargetVelLeft() - getVelocityLeft();

            rightVolts =
                ShooterConstants.kSRight +
                ShooterConstants.kVRight * getTargetVelRight() +
                ShooterConstants.kPRight * getTargetVelRight() - getVelocityRight();
        }

        leftVolts = MathUtils.clamp(leftVolts, 0, 12);
        double leftCompensatedPower = MathUtils.clamp(leftVolts * (12 / m_voltageSensor.getVoltage()), 0, 1);

        rightVolts = MathUtils.clamp(rightVolts, 0, 12);
        double rightCompensatedPower = MathUtils.clamp(rightVolts * (12 / m_voltageSensor.getVoltage()), 0, 1);

        m_shooterMotorLeft.setPower(leftCompensatedPower);
        m_shooterMotorRight.setPower(rightCompensatedPower);
    }
}
