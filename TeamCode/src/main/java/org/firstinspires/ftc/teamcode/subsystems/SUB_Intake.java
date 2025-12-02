package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SUB_Intake extends SubsystemBase {

    private final DcMotorEx m_intakeMotorRight;
    private final DcMotorEx m_intakeMotorLeft;
    private final OpMode m_OpMode;

    public SUB_Intake(OpMode p_OpMode) {
        m_OpMode = p_OpMode;

        m_intakeMotorRight = m_OpMode.hardwareMap.get(DcMotorEx.class, "intakeMotorRight");
        m_intakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_intakeMotorRight.setPower(0);
        m_intakeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_intakeMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        m_intakeMotorLeft = m_OpMode.hardwareMap.get(DcMotorEx.class, "intakeMotorLeft");
        m_intakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_intakeMotorLeft.setPower(0);
        m_intakeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_intakeMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setMotorPower(double Speed) {
        m_intakeMotorRight.setPower(Speed);
        m_intakeMotorLeft.setPower(Speed);
    }
}
