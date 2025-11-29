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
        m_dashboard = FtcDashboard.getInstance();
        m_dashboard.setTelemetryTransmissionInterval(20);
    }

    public void periodic() {
        m_shooterLeft.periodic();
//        m_shooterRight.periodic();

        m_opMode.telemetry.addData("Target Vel", "%.0f RPM", m_shooterLeft.getTargetVelocity());

        m_opMode.telemetry.addData("Vel", "%.0f RPM", m_shooterLeft.getCurrentVelocityRaw());
        m_opMode.telemetry.addData("Power", "%.3f", m_shooterLeft.getOutputPower());


        m_opMode.telemetry.addData("ready to launch?", isReadyToLaunch());

        if(Constants.ShooterConstants.kTuningMode){
            if(kPLeft != lastPLeft || kDLeft != lastDLeft || kFLeft != lastFLeft){
                m_shooterLeft.setPIDF(kPLeft, 0, kDLeft, kFLeft);

                lastPLeft = kPLeft;
                lastDLeft = kDLeft;
                lastFLeft = kFLeft;
            }

            m_shooterLeft.setTargetVelocity(velLeft);
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Target Vel", m_shooterLeft.getTargetVelocity());

        packet.put("Vel", m_shooterLeft.getCurrentVelocity());
        packet.put("Output", m_shooterLeft.getOutputPower());
        packet.put("RPM", m_shooterLeft.getCurrentVelocityRaw());


        m_dashboard.sendTelemetryPacket(packet);
    }

    public void runPower(double power){
        m_shooterLeft.runPower(power);
    }

    public void setShooterStop(){
        m_shooterLeft.setMotorsStop();
    }

    public void unjam(){
        m_shooterLeft.unjam();
    }

    public void setShootingVelocity(double velocity){
        m_shooterLeft.setTargetVelocity(velocity);
    }

    public boolean isReadyToLaunch(){
        return m_shooterLeft.getCurrentVelocity() >= (m_shooterLeft.getTargetVelocity() - Constants.ShooterConstants.kTolerance);
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