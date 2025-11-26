package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_AdjustTargetVel extends CommandBase {
    private final SUB_Shooter m_shooter;
    public CMD_AdjustTargetVel(SUB_Shooter p_shooter){
        m_shooter = p_shooter;
    }

    @Override
    public void initialize() {
        if (GlobalVariables.m_distToTag>105) m_shooter.setTargetVel(33500);
        else {
            m_shooter.setTargetVel(20500 + (105 * GlobalVariables.m_distToTag));
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}