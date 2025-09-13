package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class SUB_Shooter extends SubsystemBase {

    private final OpMode m_OpMode;
    private final DcMotorEx m_shooterMotorLeft;
    private final DcMotorEx m_shooterMotorRight;
    private final CRServo m_kickerRight;
    private final CRServo m_kickerLeft;

    public static double shooterP = 0;
    public static double shooterD = 0;
    public static double shooterF = 0;
    public static int shooterVelocity = 0;

    private final boolean tuning = true;


    public SUB_Shooter(OpMode p_OpMode) {

        m_OpMode = p_OpMode;
        m_shooterMotorLeft = m_OpMode.hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        m_shooterMotorRight = m_OpMode.hardwareMap.get(DcMotorEx.class, "shooterMotorRight");
        m_kickerLeft = m_OpMode.hardwareMap.get(CRServo.class, "kickerLeft");
        m_kickerRight = m_OpMode.hardwareMap.get(CRServo.class, "kickerRight");
        m_shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setVelocity(double velocity) {
        m_shooterMotorLeft.setVelocity(velocity, AngleUnit.DEGREES);
        m_shooterMotorRight.setVelocity(velocity, AngleUnit.DEGREES);
    }

    public void kickRight(double power) {
        m_kickerRight.setPower(power);
    }

    public void kickLeft(double power) {
        m_kickerLeft.setPower(power);
    }

    @Override
    public void periodic() {
        if (tuning) {
            m_shooterMotorLeft.setVelocityPIDFCoefficients(shooterP, 0, shooterD, shooterF);
            m_shooterMotorRight.setVelocityPIDFCoefficients(shooterP, 0, shooterD, shooterF);

            m_shooterMotorLeft.setVelocity(shooterVelocity, AngleUnit.DEGREES);
            m_shooterMotorRight.setVelocity(shooterVelocity, AngleUnit.DEGREES);
        }
    }
}
