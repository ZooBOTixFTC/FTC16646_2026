package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SUB_Lift extends SubsystemBase {
    private final DcMotor m_LiftMotor;
    private final OpMode m_opMode;

    public SUB_Lift(OpMode p_opmode) {
       m_opMode = p_opmode;
       m_LiftMotor = m_opMode.hardwareMap.get(DcMotor.class, "liftMotor");
       m_LiftMotor.setPower(0);
       m_LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       m_LiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
       m_LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       m_LiftMotor.setTargetPosition(m_LiftMotor.getCurrentPosition());
       m_LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTargetPosition(int targetPos){
        m_LiftMotor.setTargetPosition(targetPos);
        m_LiftMotor.setPower(1);
    }
}
