package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

@Config
public class SUB_Shooter extends SubsystemBase {
    private final OpMode m_opMode;
    private final DcMotorEx m_shooterMotorLeft;
    private final DcMotorEx m_shooterMotorRight;
    private final VoltageSensor m_voltageSensor;
    private final FtcDashboard m_dashboard;
    private double m_targetVelLeft, m_targetVelRight;
    private double lastLeftVelError, lastRightVelError = 0;

    public static double leftP, leftD, leftV, leftS, leftVel, rightP, rightD, rightV, rightS, rightVel;

    private int lastLeftPos = 0;
    private int lastRightPos = 0;
    private long lastTimeLeft, lastTimeRight = System.nanoTime();
    private double smoothedVelocityLeft, smoothedVelocityRight = 0;

    public SUB_Shooter(OpMode p_opMode) {
        m_opMode = p_opMode;
        m_shooterMotorLeft = m_opMode.hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        m_shooterMotorRight = m_opMode.hardwareMap.get(DcMotorEx.class, "shooterMotorRight");

        m_voltageSensor = m_opMode.hardwareMap.voltageSensor.iterator().next();

        m_dashboard = FtcDashboard.getInstance();
        m_dashboard.setTelemetryTransmissionInterval(20);

        m_shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_shooterMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        m_shooterMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        m_shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setTargetVelLeft(double velocity) {
        m_targetVelLeft = velocity;
    }

    public void setTargetVelRight(double velocity) {
        m_targetVelRight = velocity;
    }
    public void setTargetVel(double velocity) {
        m_targetVelLeft = velocity;
        m_targetVelRight = velocity;
    }

    public double getVelocityLeft() {
        long currentTime = System.nanoTime();
        double deltaTimeSec = (currentTime - lastTimeLeft) / 1e9; // convert nanoseconds to milliseconds

        if (deltaTimeSec >= 0.02) { // 20ms loop
            int currentPos = m_shooterMotorLeft.getCurrentPosition();
            int deltaTicks = currentPos - lastLeftPos;

            double ticksPerRev = 28.0; // 1:1 GoBilda motor
            double revs = deltaTicks / ticksPerRev;
            double rawVelocity = (revs * 360.0) / deltaTimeSec;

            smoothedVelocityLeft = ShooterConstants.kAlpha * rawVelocity + (1 - ShooterConstants.kAlpha) * smoothedVelocityLeft;

            lastLeftPos = currentPos;
            lastTimeLeft = currentTime;
        }

        return Math.round(smoothedVelocityLeft);
    }

    public double getVelocityRight() {
        long currentTime = System.nanoTime();
        double deltaTimeSec = (currentTime - lastTimeRight) / 1e9; // convert nanoseconds to milliseconds

        if (deltaTimeSec >= 0.02) { // 20ms loop
            int currentPos = m_shooterMotorRight.getCurrentPosition();
            int deltaTicks = currentPos - lastRightPos;

            double ticksPerRev = 28.0; // 1:1 GoBilda motor
            double revs = deltaTicks / ticksPerRev;
            double rawVelocity = (revs * 360.0) / deltaTimeSec;

            smoothedVelocityRight = ShooterConstants.kAlpha * rawVelocity + (1 - ShooterConstants.kAlpha) * smoothedVelocityRight;

            lastRightPos = currentPos;
            lastTimeRight = currentTime;
        }

        return Math.round(smoothedVelocityRight);
    }

    public double getTargetVelLeft(){
        return m_targetVelLeft;
    }

    public double getTargetVelRight(){
        return m_targetVelRight;
    }

    @Override
    public void periodic() {
        m_opMode.telemetry.addData("Shooter Vel Left", getVelocityLeft());
        m_opMode.telemetry.addData("Shooter Target Vel Left", getTargetVelLeft());
        m_opMode.telemetry.addData("Shooter Vel Right", getVelocityRight());
        m_opMode.telemetry.addData("Shooter Target Vel Right", getTargetVelRight());

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("left vel", getVelocityLeft());
        packet.put("left target vel", getTargetVelLeft());
        packet.put("right vel", getVelocityRight());
        packet.put("right target vel", getTargetVelRight());

        m_dashboard.sendTelemetryPacket(packet);

        double leftVolts, rightVolts;

        double leftVelError = getTargetVelLeft() - getVelocityLeft();
        double rightVelError = getTargetVelRight() - getVelocityRight();

        double leftErrorRate = (leftVelError - lastLeftVelError) / .02;
        double rightErrorRate = (rightVelError - lastRightVelError) / .02;

        lastLeftVelError = leftVelError;
        lastRightVelError = rightVelError;

        if(ShooterConstants.kTuningMode){
            setTargetVelLeft(leftVel);
            setTargetVelRight(rightVel);

            leftVolts =
                leftS +
                leftV * getTargetVelLeft() +
                leftP * (getTargetVelLeft() - getVelocityLeft()) +
                leftD * leftErrorRate;

            rightVolts =
                rightS +
                rightV * getTargetVelRight() +
                rightP * (getTargetVelRight() - getVelocityRight()) +
                rightD * rightErrorRate;
        }else{
            leftVolts =
                ShooterConstants.kSLeft +
                ShooterConstants.kVLeft * getTargetVelLeft() +
                ShooterConstants.kPLeft * (getTargetVelLeft() - getVelocityLeft()) +
                ShooterConstants.kDLeft * leftErrorRate;

            rightVolts =
                ShooterConstants.kSRight +
                ShooterConstants.kVRight * getTargetVelRight() +
                ShooterConstants.kPRight * (getTargetVelRight() - getVelocityRight()) +
                ShooterConstants.kDRight * rightErrorRate;
        }

        double leftCompensatedPower = MathUtils.clamp(leftVolts * (12 / m_voltageSensor.getVoltage()), -1, 1);
        double rightCompensatedPower = MathUtils.clamp(rightVolts * (12 / m_voltageSensor.getVoltage()), -1, 1);

        if(m_targetVelLeft == 0) leftCompensatedPower = 0;
        if(m_targetVelRight == 0) rightCompensatedPower = 0;

        m_shooterMotorLeft.setPower(leftCompensatedPower);
        m_shooterMotorRight.setPower(rightCompensatedPower);
    }
}
