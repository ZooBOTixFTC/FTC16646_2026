package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

@Config
public class SUB_Shooter extends SubsystemBase {
    private final LinearMotion m_shooterLeft;
//    private final LinearMotion m_shooterRight;
    private final FtcDashboard m_dashboard;
    private final OpMode m_opMode;
    
    public static double kPLeft, kDLeft, kFLeft, velLeft = 0;
    private double lastPLeft, lastDLeft, lastFLeft;

//    public static double kPRight, kDRight, kFRight, velRight = 0;
//    private double lastPRight, lastDRight, lastFRight;

    private final Servo m_stopperLeft, m_stopperRight;

    public SUB_Shooter(OpMode p_opMode){
        m_opMode = p_opMode;

        m_stopperLeft = m_opMode.hardwareMap.get(Servo.class, "stopperLeft");
        m_stopperRight = m_opMode.hardwareMap.get(Servo.class, "stopperRight");

        m_stopperRight.setDirection(Servo.Direction.FORWARD);
        m_stopperLeft.setDirection(Servo.Direction.REVERSE);

        this.m_shooterLeft = new LinearMotion(
                "shooter",
                new DcMotorEx[]{
                    m_opMode.hardwareMap.get(DcMotorEx.class,"shooterMotorLeft"),
                    m_opMode.hardwareMap.get(DcMotorEx.class, "shooterMotorRight")
                },
                new boolean[]{false, true},
                m_opMode.hardwareMap.get(DcMotorEx.class,"shooterMotorLeft"),
                false,
                Constants.ShooterConstants.kMaxVel,
                Constants.ShooterConstants.kP,
                0.0,
                0.0,
                Constants.ShooterConstants.kF
        );

//        this.m_shooterRight = new LinearMotion(
//                "shooter",
//                new DcMotorEx[]{
//                    m_opMode.hardwareMap.get(DcMotorEx.class,"shooterMotorRight")
//                },
//                new boolean[]{true},
//                m_opMode.hardwareMap.get(DcMotorEx.class,"shooterMotorRight"),
//                true,
//                Constants.ShooterConstants.kMaxVel,
//                Constants.ShooterConstants.kP,
//                0.0,
//                0.0,
//                Constants.ShooterConstants.kFRight
//        );

        m_dashboard = FtcDashboard.getInstance();
        m_dashboard.setTelemetryTransmissionInterval(20);
    }

    public void periodic() {
        m_shooterLeft.periodic();
//        m_shooterRight.periodic();

        m_opMode.telemetry.addData("Target Vel", "%.0f RPM", m_shooterLeft.getTargetVelocity());

        m_opMode.telemetry.addData("Left Vel", "%.0f RPM", m_shooterLeft.getCurrentVelocityRaw());
        m_opMode.telemetry.addData("Left Power", "%.3f", m_shooterLeft.getOutputPower());

//        m_opMode.telemetry.addData("Right Vel", "%.0f RPM", m_shooterRight.getCurrentVelocityRaw());
//        m_opMode.telemetry.addData("Right Power", "%.3f", m_shooterRight.getOutputPower());

        m_opMode.telemetry.addData("ready to launch?", isReadyToLaunch());

        if(Constants.ShooterConstants.kTuningMode){
            if(kPLeft != lastPLeft || kDLeft != lastDLeft || kFLeft != lastFLeft){
                m_shooterLeft.setPIDF(kPLeft, 0, kDLeft, kFLeft);

                lastPLeft = kPLeft;
                lastDLeft = kDLeft;
                lastFLeft = kFLeft;
            }

//            if(kPRight != lastPRight || kDRight != lastDRight || kFRight != lastFRight){
//                m_shooterRight.setPIDF(kPRight, 0, kDRight, kFRight);
//
//                lastPRight = kPRight;
//                lastDRight = kDRight;
//                lastFRight = kFRight;
//            }

            m_shooterLeft.setTargetVelocity(velLeft);
//            m_shooterRight.setTargetVelocity(velRight);
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Target Vel", m_shooterLeft.getTargetVelocity());

        packet.put("Left Vel", m_shooterLeft.getCurrentVelocity());
        packet.put("Left Output", m_shooterLeft.getOutputPower());
        packet.put("Left RPM", m_shooterLeft.getCurrentVelocityRaw());

//        packet.put("Right Vel", m_shooterRight.getCurrentVelocity());
//        packet.put("Right Output", m_shooterRight.getOutputPower());
//        packet.put("Right RPM", m_shooterRight.getCurrentVelocityRaw());

        m_dashboard.sendTelemetryPacket(packet);
    }

    public void runPower(double power){
        m_shooterLeft.runPower(power);
//        m_shooterRight.runPower(power);
    }

    public void setShooterStop(){
        m_shooterLeft.setMotorsStop();
//        m_shooterRight.setMotorsStop();
    }

    public void unjam(){
        m_shooterLeft.unjam();
//        m_shooterRight.unjam();
    }

    public void setShootingVelocity(double velocity){
        m_shooterLeft.setTargetVelocity(velocity);
//        m_shooterRight.setTargetVelocity(velocity);
    }

    public boolean isReadyToLaunch(){
        return m_shooterLeft.getCurrentVelocity() >= (m_shooterLeft.getTargetVelocity() - Constants.ShooterConstants.kTolerance);
//                && (m_shooterRight.getCurrentVelocity() >= m_shooterRight.getTargetVelocity() - Constants.ShooterConstants.kTolerance);
    }

    public void setStopperClosed(){
        m_stopperLeft.setPosition(100.0/300.0);
        m_stopperRight.setPosition(100.0/300.0);
    }

    public void setStopperOpen(){
        m_stopperLeft.setPosition(0);
        m_stopperRight.setPosition(0);
    }
}