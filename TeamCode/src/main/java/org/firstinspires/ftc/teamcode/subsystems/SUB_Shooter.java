package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

@Config
public class SUB_Shooter extends SubsystemBase {
    private final OpMode m_opMode;
    private final DcMotorEx m_shooterMotorLeft;
    private final DcMotorEx m_shooterMotorRight;
    private final CRServo m_kickerRight;
    private final CRServo m_kickerLeft;
    private final CRServo m_feederLeft;
    private final CRServo m_feederRight;

    private double m_targetVelocity;

    public static double shooterP, shooterD, shooterF, shooterVel, alpha = 0.2;
    private double lastP, lastD, lastF, lastVelocity = 0;

    private int lastPos = 0;
    private long lastTime = System.nanoTime();
    private double smoothedVelocity = 0;

    private final FtcDashboard dashboard;

    public SUB_Shooter(OpMode p_opMode) {
        m_opMode = p_opMode;
        m_shooterMotorLeft = m_opMode.hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        m_shooterMotorRight = m_opMode.hardwareMap.get(DcMotorEx.class, "shooterMotorRight");

        m_kickerLeft = m_opMode.hardwareMap.get(CRServo.class, "kickerLeft");
        m_kickerRight = m_opMode.hardwareMap.get(CRServo.class, "kickerRight");
        m_feederRight = m_opMode.hardwareMap.get(CRServo.class, "feederRight");
        m_feederLeft = m_opMode.hardwareMap.get(CRServo.class, "feederLeft");

        m_feederRight.setDirection(DcMotorSimple.Direction.REVERSE);
        m_kickerRight.setDirection(DcMotorSimple.Direction.FORWARD);
        m_feederLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        m_kickerLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        m_shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m_shooterMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                ShooterConstants.kShooterFarP, 0, ShooterConstants.kShooterD, ShooterConstants.kShooterFarF
        ));
        m_shooterMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                ShooterConstants.kShooterFarP, 0, ShooterConstants.kShooterD, ShooterConstants.kShooterFarF
        ));

        m_shooterMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        m_shooterMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        m_shooterMotorLeft.setVelocity(0, AngleUnit.DEGREES);
        m_shooterMotorRight.setVelocity(0, AngleUnit.DEGREES);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(20);
    }

    public void setVelocity(double velocity) {
        m_targetVelocity = velocity;
        m_shooterMotorLeft.setVelocity(velocity, AngleUnit.DEGREES);
        m_shooterMotorRight.setVelocity(velocity, AngleUnit.DEGREES);
    }

    public double getVelocity() {
        long currentTime = System.nanoTime();
        double deltaTimeSec = (currentTime - lastTime) / 1e9;

        if (deltaTimeSec >= 0.02) { // 20ms loop
            int currentPos = m_shooterMotorLeft.getCurrentPosition();
            int deltaTicks = currentPos - lastPos;

            double ticksPerRev = 28.0; // adjust if needed
            double revs = deltaTicks / ticksPerRev;
            double rawVelocity = (revs * 360.0) / deltaTimeSec;

            smoothedVelocity = alpha * rawVelocity + (1 - alpha) * smoothedVelocity;

            lastPos = currentPos;
            lastTime = currentTime;
        }

        return smoothedVelocity;
    }

    public double getTargetVelocity(){
        return m_targetVelocity;
    }

    public void setKickRightPower(double power) {
        m_kickerRight.setPower(power);
    }

    public void setKickLeftPower(double power) {
        m_kickerLeft.setPower(power);
    }
    public void setFeederPower(double power) {
        m_feederLeft.setPower(power);
        m_feederRight.setPower(power);
    }

    @Override
    public void periodic() {
        m_opMode.telemetry.addData("Shooter Vel", getVelocity());
        m_opMode.telemetry.addData("Shooter Target Vel", getTargetVelocity());
        m_opMode.telemetry.addData("Shooter Error", Math.abs(getVelocity() - getTargetVelocity()));

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("targetVel", getTargetVelocity());
        packet.put("currentVel", getVelocity());
        packet.put("ShooterError", Math.abs(getVelocity() - getTargetVelocity()));

        dashboard.sendTelemetryPacket(packet);

        if(ShooterConstants.kTuningMode){
            setKickRightPower(1);
            setKickLeftPower(1);
            setFeederPower(1);

            if (shooterP != lastP ||
                    shooterD != lastD ||
                    shooterF != lastF) {

                m_shooterMotorLeft.setVelocityPIDFCoefficients(
                        shooterP, 0, shooterD, shooterF);
                m_shooterMotorRight.setVelocityPIDFCoefficients(
                        shooterP, 0, shooterD, shooterF);

                lastP = shooterP;
                lastD = shooterD;
                lastF = shooterF;
            }

            if(shooterVel != lastVelocity){
                setVelocity(shooterVel);

                lastVelocity = shooterVel;
            }
        }
    }
}
