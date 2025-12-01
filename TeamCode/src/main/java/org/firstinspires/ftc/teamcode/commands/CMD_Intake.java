package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Intake;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Turntable;

public class CMD_Intake extends CommandBase {
    private final SUB_Turntable m_turntable;
    private final GamepadEx m_gamepad;
    private final SUB_Intake m_intake;
    private boolean canBeCancelled;

    private final ElapsedTime m_timer = new ElapsedTime();

    public CMD_Intake(SUB_Turntable p_turntable, SUB_Intake p_intake, GamepadEx gamepad){
        m_turntable = p_turntable;
        m_intake = p_intake;
        m_gamepad = gamepad;
        addRequirements(p_turntable,p_intake);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        canBeCancelled = false;
        GlobalVariables.m_intakeOn = true;
    }

    @Override
    public void execute(){
        if (m_gamepad.gamepad.left_trigger < 0.2) canBeCancelled = true;
        if (GlobalVariables.m_intakeOn) {
            m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn);
            if (m_timer.milliseconds() >= 500) {
                m_turntable.intakeRotateRight();
                m_timer.reset();

            }
        }
    }

    @Override
    public boolean isFinished() {
        return m_gamepad.gamepad.left_trigger>0.5 && m_timer.milliseconds()>100 && canBeCancelled;
    }
    @Override
    public void end(boolean interrupted) {
        m_turntable.stopMotor();
        m_turntable.rotateRight();
        m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff);
        GlobalVariables.m_intakeOn = false;
    }
}
