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

import org.firstinspires.ftc.teamcode.Constants;

@Config
public class SUB_Shooter extends SubsystemBase {
    private final LinearMotion m_shooterLeft;
    private final LinearMotion m_shooterRight;
    private final FtcDashboard m_dashboard;
    private final OpMode m_opMode;
    
    public static double kPLeft, kDLeft, kFLeft, velLeft = 0;
    private double lastPLeft, lastDLeft, lastFLeft;

    public static double kPRight, kDRight, kFRight, velRight = 0;
    private double lastPRight, lastDRight, lastFRight;

    public SUB_Shooter(OpMode p_opMode){
        m_opMode = p_opMode;
        
        this.m_shooterLeft = new LinearMotion(
                "shooter",
                new DcMotorEx[]{
                    m_opMode.hardwareMap.get(DcMotorEx.class,"shooterMotorLeft")
                },
                new boolean[]{false},
                m_opMode.hardwareMap.get(DcMotorEx.class,"shooterMotorLeft"),
                false,
                2600,
                Constants.ShooterConstants.kP,
                0.0,
                0.0,
                Constants.ShooterConstants.kFLeft
        );

        this.m_shooterRight = new LinearMotion(
                "shooter",
                new DcMotorEx[]{
                    m_opMode.hardwareMap.get(DcMotorEx.class,"shooterMotorRight")
                },
                new boolean[]{true},
                m_opMode.hardwareMap.get(DcMotorEx.class,"shooterMotorRight"),
                true,
                2600,
                Constants.ShooterConstants.kP,
                0.0,
                0.0,
                Constants.ShooterConstants.kFRight
        );

        m_dashboard = FtcDashboard.getInstance();
        m_dashboard.setTelemetryTransmissionInterval(20);
    }

//    @Override
    public void periodic() {
        m_shooterLeft.periodic();
        m_shooterRight.periodic();
        m_opMode.telemetry.addData("=== SHOOTER ===", "");
        m_opMode.telemetry.addData("Target Vel", "%.0f RPM", m_shooterLeft.getTargetVelocity());
        m_opMode.telemetry.addData("Left Vel", "%.0f RPM", m_shooterLeft.getCurrentVelocityRaw());
        m_opMode.telemetry.addData("Right Power", "%.3f", m_shooterLeft.getOutputPower());

        m_opMode.telemetry.addData("Right Vel", "%.0f RPM", m_shooterRight.getCurrentVelocityRaw());
        m_opMode.telemetry.addData("Left Power", "%.3f", m_shooterRight.getOutputPower());

        m_opMode.telemetry.addData("===== SHOOTER STATUS =====", "");
        m_opMode.telemetry.addData("ready to short shoot?", isReadyToShortLaunch());
        m_opMode.telemetry.addData("ready to far shoot?", isReadyToFarLaunch());

        if(Constants.ShooterConstants.kTuningMode){
            if(kPLeft != lastPLeft || kDLeft != lastDLeft || kFLeft != lastFLeft){
                m_shooterLeft.setPIDF(kPLeft, 0, kDLeft, kFLeft);

                lastPLeft = kPLeft;
                lastDLeft = kDLeft;
                lastFLeft = kFLeft;
            }

            if(kPRight != lastPRight || kDRight != lastDRight || kFRight != lastFRight){
                m_shooterRight.setPIDF(kPRight, 0, kDRight, kFRight);

                lastPRight = kPRight;
                lastDRight = kDRight;
                lastFRight = kFRight;
            }

            m_shooterLeft.setTargetVelocity(velLeft);
            m_shooterRight.setTargetVelocity(velRight);
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Target Vel", m_shooterLeft.getTargetVelocity());

        packet.put("Left Vel", m_shooterLeft.getCurrentVelocity());
        packet.put("Left Output", m_shooterLeft.getOutputPower());
        packet.put("Left RPM", m_shooterLeft.getCurrentVelocityRaw());

        packet.put("Right Vel", m_shooterRight.getCurrentVelocity());
        packet.put("Right Output", m_shooterRight.getOutputPower());
        packet.put("Right RPM", m_shooterRight.getCurrentVelocityRaw());

        m_dashboard.sendTelemetryPacket(packet);
    }

    public void setShooterStop(){
        m_shooterLeft.setMotorsStop();
        m_shooterRight.setMotorsStop();
    }

    public void setShootingVelocity(double velocity){
        m_shooterLeft.setTargetVelocity(velocity);
        m_shooterRight.setTargetVelocity(velocity);
    }

    public Command m_shooterStop(){
        return new InstantCommand(m_shooterLeft::setMotorsStop)
                .alongWith(new InstantCommand(m_shooterRight::setMotorsStop));
    }

    public Command m_shooterFarLaunch(){
        return new RunCommand(() -> m_shooterLeft.setTargetVelocity(Constants.ShooterConstants.kMaxVel));
    }

    public boolean isReadyToFarLaunch(){
        return m_shooterLeft.getCurrentVelocity() >= Constants.ShooterConstants.kMaxVel
                && m_shooterRight.getCurrentVelocity() >= Constants.ShooterConstants.kMaxVel;
    }

    public boolean isReadyToShortLaunch(){
        return m_shooterLeft.getCurrentVelocity() >= Constants.ShooterConstants.kMidFieldVel
                && m_shooterRight.getCurrentVelocity() >= Constants.ShooterConstants.kMidFieldVel;
    }

    public boolean isReadyToLaunch(){
        return m_shooterLeft.getCurrentVelocity() >= m_shooterLeft.getTargetVelocity()
                && m_shooterRight.getCurrentVelocity() >= m_shooterRight.getTargetVelocity();
    }
}