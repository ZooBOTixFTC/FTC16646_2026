package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.GlobalVariables;

@Config
public class SUB_Shooter extends SubsystemBase {
    private Servo m_KickerServo;
    private final OpMode m_opMode;
    private final DcMotorEx m_shooterMotor;

    private double m_targetVelocity;

    public static double shooterP, shooterD, shooterF, lastP, lastD, lastF = 0;
    public static double shooterVelocity, lastVelocity = 0;

    private int lastPos = 0;
    private long lastTime = System.nanoTime();
    private double smoothedVelocity = 0;

    private final FtcDashboard dashboard;

    public SUB_Shooter(OpMode p_opMode) {

        m_opMode = p_opMode;
        m_shooterMotor = m_opMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");
        m_KickerServo = m_opMode.hardwareMap.get(Servo.class, "kickServo");
        m_KickerServo.setDirection(Servo.Direction.REVERSE);


        m_shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                ShooterConstants.kShooterP, 0, ShooterConstants.kShooterD, ShooterConstants.kShooterFarF
        ));

        m_shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        m_shooterMotor.setVelocity(0, AngleUnit.DEGREES);
        m_shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(20);
    }
    public void setKickerPosition(double position){
        m_KickerServo.setPosition(position);
    }

    public void setVelocity(double velocity) {
//        m_shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        m_shooterMotor.setPower(1);
        m_targetVelocity = velocity;
        m_shooterMotor.setVelocity(velocity, AngleUnit.DEGREES);
    }

    public double getVelocity() {
        long currentTime = System.nanoTime();
        double deltaTimeSec = (currentTime - lastTime) / 1e9;

        if (deltaTimeSec >= 0.3) { // 100ms
            int currentPos = m_shooterMotor.getCurrentPosition();
            int deltaTicks = currentPos - lastPos;

            double ticksPerRev = 28.0; // adjust if needed
            double revs = deltaTicks / ticksPerRev;
            smoothedVelocity = (revs * 360.0) / deltaTimeSec;

            lastPos = currentPos;
            lastTime = currentTime;
        }

        return smoothedVelocity;
    }

    public double getTargetVelocity(){
        return m_targetVelocity;
    }

    @Override
    public void periodic() {
        m_opMode.telemetry.addData("Shooter vel", getVelocity());

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("targetVel", getTargetVelocity());
        packet.put("currentVel", getVelocity());

        dashboard.sendTelemetryPacket(packet);

        if(ShooterConstants.kTuningMode){
            if (shooterP != lastP ||
                    shooterD != lastD ||
                    shooterF != lastF) {

                m_shooterMotor.setVelocityPIDFCoefficients(
                        shooterP, 0, shooterD, shooterF);

                lastP = shooterP;
                lastD = shooterD;
                lastF = shooterF;
            }

            if(shooterVelocity != lastVelocity){
                m_shooterMotor.setVelocity(shooterVelocity, AngleUnit.DEGREES);

                lastVelocity = shooterVelocity;
            }
        }
    }
}
