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
        m_shooter.setTargetVel(22000 + (100 * GlobalVariables.m_distToTag));
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}