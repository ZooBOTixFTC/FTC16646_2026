package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Constants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SUB_Shooter extends SubsystemBase {


    private final OpMode m_OpMode;
    private final DcMotorEx m_shooterMotorLeft;
    private final DcMotorEx m_shooterMotorRight;
    private final ServoEx m_kickerRight;
    private final ServoEx m_kickerLeft;

    public SUB_Shooter(OpMode p_OpMode) {

        m_OpMode = p_OpMode;
        m_shooterMotorLeft = m_OpMode.hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        m_shooterMotorRight = m_OpMode.hardwareMap.get(DcMotorEx.class, "shooterMotorRight");
        m_kickerLeft = m_OpMode.hardwareMap.get(ServoEx.class, "kickerLeft");
        m_kickerRight = m_OpMode.hardwareMap.get(ServoEx.class, "kickerRight");
        m_shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void readyToShoot(double power) {

        m_shooterMotorRight.setPower(power);
        m_shooterMotorLeft.setPower(power);
    }

    public void kickRightServo(double pos) {

        m_kickerRight.setPosition(pos/300);
    }

    public void kickLeft(double pos) {

        m_kickerLeft.setPosition(pos/300);
    }


    @Override
    public void periodic() {
        telemetry.addData("kickerLeft", m_kickerLeft.getPosition());
        telemetry.addData("kickerRight", m_kickerRight.getPosition());
        telemetry.addData("Shooter Speed", m_shooterMotorRight.getVelocity(AngleUnit.DEGREES));

    }
}
