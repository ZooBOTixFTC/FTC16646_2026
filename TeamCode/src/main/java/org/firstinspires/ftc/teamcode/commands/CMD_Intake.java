package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.SUB_Turntable;

public class CMD_Intake extends CommandBase {
    private final SUB_Turntable m_turntable;
    private final ElapsedTime m_timer = new ElapsedTime();

    public CMD_Intake(SUB_Turntable p_turntable){
        m_turntable = p_turntable;
        addRequirements(m_turntable);
    }

    @Override
    public void initialize(){
        m_timer.reset();
    }

    @Override
    public void execute(){
        if(m_timer.milliseconds() < 670){
            m_turntable.rotateRight();
        }else{
            m_timer.reset();
        }
    }
}
