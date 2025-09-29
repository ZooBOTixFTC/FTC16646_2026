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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.LinearInterpolator;

@Config
public class SUB_Shooter extends SubsystemBase {
    private final OpMode m_opMode;
    private final DcMotorEx m_shooterMotorLeft;
    private final DcMotorEx m_shooterMotorRight;
    private final CRServo m_kickerRight;
    private final CRServo m_kickerLeft;
    private final LinearInterpolator m_shooterInterpolator;

    private double m_targetVelocity;

    public static double shooterP = 0;
    public static double shooterD = 0;
    public static double shooterF = 0;
    public static double shooterVelocity = 0;

    private final FtcDashboard dashboard;

    public SUB_Shooter(OpMode p_opMode) {
        m_opMode = p_opMode;
        m_shooterMotorLeft = m_opMode.hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        m_shooterMotorRight = m_opMode.hardwareMap.get(DcMotorEx.class, "shooterMotorRight");

        m_kickerLeft = m_opMode.hardwareMap.get(CRServo.class, "kickerLeft");
        m_kickerRight = m_opMode.hardwareMap.get(CRServo.class, "kickerRight");

        m_shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        m_shooterMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
//                ShooterConstants.kShooterP, 0, ShooterConstants.kShooterD, ShooterConstants.kShooterF
//        ));
//        m_shooterMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
//                ShooterConstants.kShooterP, 0, ShooterConstants.kShooterD, ShooterConstants.kShooterF
//        ));

        m_shooterMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        m_shooterMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        m_shooterMotorLeft.setVelocity(0, AngleUnit.DEGREES);
        m_shooterMotorRight.setVelocity(0, AngleUnit.DEGREES);

        m_shooterMotorLeft.setPower(0);
        m_shooterMotorRight.setPower(0);

        m_shooterInterpolator = new LinearInterpolator(ShooterConstants.kShooterInterpolationTable);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(20);
    }

    public void setVelocity(int velocity) {
        m_targetVelocity = velocity;
        m_shooterMotorLeft.setVelocity(velocity, AngleUnit.DEGREES);
        m_shooterMotorRight.setVelocity(velocity, AngleUnit.DEGREES);

        m_shooterMotorLeft.setPower(1);
        m_shooterMotorRight.setPower(1);
    }

    public void setVelocity (double distance){
        int interpolatedVelocity = (int)m_shooterInterpolator.getInterpolatedValue(distance);
        setVelocity(interpolatedVelocity);
    }

    public double getVelocity (){
        return m_shooterMotorLeft.getVelocity(AngleUnit.DEGREES);
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

    @Override
    public void periodic() {
        if(ShooterConstants.kTuningMode){
            m_shooterMotorLeft.setVelocityPIDFCoefficients(shooterP, 0, shooterD, shooterF);
            m_shooterMotorRight.setVelocityPIDFCoefficients(shooterP, 0, shooterD, shooterF);

            m_shooterMotorLeft.setVelocity(shooterVelocity, AngleUnit.DEGREES);
            m_shooterMotorRight.setVelocity(shooterVelocity, AngleUnit.DEGREES);

            TelemetryPacket packet = new TelemetryPacket();

            m_targetVelocity = shooterVelocity;

            packet.put("targetVel", getTargetVelocity());
            packet.put("currentVel", getVelocity());

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
