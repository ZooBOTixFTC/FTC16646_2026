package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

@Config
public class SUB_Shooter extends SubsystemBase {
    private final OpMode m_opMode;
    private final DcMotorEx m_shooterMotorLeft;
    private final DcMotorEx m_shooterMotorRight;
    private final Servo m_kickerRight;
    private final Servo m_kickerLeft;
    private final VoltageSensor m_voltageSensor;
    private double m_targetVelocity;

    public static double shooterP, shooterV, shooterS, shooterVel, alpha = 0.2;
    private double lastP, lastV, lastS, lastVelocity = 0;

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

        m_shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m_shooterMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                .15, 0, 0, .5));
        m_shooterMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                .15, 0, 0, .5));

        m_shooterMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        m_shooterMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        m_shooterMotorLeft.setVelocity(0, AngleUnit.DEGREES);
        m_shooterMotorRight.setVelocity(0, AngleUnit.DEGREES);
    }

    public void setTargetVel(double velocity) {
        m_targetVelocity = velocity;

        m_shooterMotorLeft.setVelocity(velocity, AngleUnit.DEGREES);
        m_shooterMotorRight.setVelocity(velocity, AngleUnit.DEGREES);
    }

    public double getVelocity() {
        long currentTime = System.nanoTime();
        double deltaTimeSec = (currentTime - lastTime) / 1e9; // convert nanoseconds to milliseconds

        if (deltaTimeSec >= 0.02) { // 20ms loop
            int currentPos = m_shooterMotorLeft.getCurrentPosition();
            int deltaTicks = currentPos - lastPos;

            double ticksPerRev = 28.0;
            double revs = deltaTicks / ticksPerRev;
            double rawVelocity = (revs * 360.0) / deltaTimeSec;

            smoothedVelocity = alpha * rawVelocity + (1 - alpha) * smoothedVelocity;

            lastPos = currentPos;
            lastTime = currentTime;
        }

        return Math.round(smoothedVelocity);
    }

    public double getTargetVel(){
        return m_targetVelocity;
    }

    public void setKickRightPos(double angDeg) {
        m_kickerRight.setPosition(angDeg / 300);
    }

    public void setKickLeftPos(double angDeg) {
        m_kickerLeft.setPosition(angDeg / 300);
    }


    @Override
    public void periodic() {
        m_opMode.telemetry.addData("Shooter Vel", getVelocity());
        m_opMode.telemetry.addData("Shooter Target Vel", getTargetVel());
        m_opMode.telemetry.addData("Shooter Error", Math.abs(getVelocity() - getTargetVel()));

//        if(ShooterConstants.kTuningMode){
//            if (shooterP != lastP ||
//                    shooterV != lastV ||
//                    shooterS != lastS) {
//
//                lastP = shooterP;
//                lastV = shooterV;
//                lastS = shooterS;
//            }
//
//            if(shooterVel != lastVelocity){
//                setVelocity(shooterVel);
//
//                lastVelocity = shooterVel;
//            }
//        }

//        double volts =
//            ShooterConstants.kS +
//            ShooterConstants.kV * m_targetVelocity +
//            ShooterConstants.kP * (getTargetVelocity() - getVelocity());
//
//        volts = MathUtils.clamp(volts, 0, 12);
//        double compensatedPower = MathUtils.clamp(volts * (12 / m_voltageSensor.getVoltage()), -1, 1);
//
//        m_shooterMotorLeft.setPower(compensatedPower);
//        m_shooterMotorRight.setPower(compensatedPower);
    }
}
