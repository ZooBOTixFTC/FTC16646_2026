package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.LiftConstants;

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

    public void setTargetPos(double angDeg){
        double targetTicks = (angDeg/360) * LiftConstants.kTicksPerRev;
        m_LiftMotor.setTargetPosition((int) targetTicks);
        m_LiftMotor.setPower(1);
    }

    public double getPos(){
        return (m_LiftMotor.getCurrentPosition() / LiftConstants.kTicksPerRev) * 360;
    }

    @Override
    public void periodic(){
        m_opMode.telemetry.addData("lift pos", getPos());
    }
}
