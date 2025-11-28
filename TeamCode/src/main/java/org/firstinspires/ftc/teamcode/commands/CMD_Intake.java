package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Turntable;

public class CMD_Intake extends CommandBase {
    private final SUB_Turntable m_turntable;
    private final ElapsedTime m_timer = new ElapsedTime();

    public CMD_Intake(SUB_Turntable p_turntable){
        m_turntable = p_turntable;
        addRequirements(p_turntable);
    }

    @Override
    public void initialize(){
        m_timer.reset();
    }

    @Override
    public void execute(){
        if (GlobalVariables.m_intakeOn) {
            if (m_timer.milliseconds() >= 500) {
                m_turntable.intakeRotateRight();
                m_timer.reset();

            }
        }
    }

    @Override
    public boolean isFinished() {
        return !GlobalVariables.m_intakeOn;
    }
    @Override
    public void end(boolean interrupted) {
        m_turntable.stopMotor();
    }
}
