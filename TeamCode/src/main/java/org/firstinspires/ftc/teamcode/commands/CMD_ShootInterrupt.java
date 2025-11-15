package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShootInterrupt extends CommandBase {
    private final SUB_Shooter m_shooter;
    public CMD_ShootInterrupt(SUB_Shooter p_shooter){
        m_shooter = p_shooter;

        addRequirements(m_shooter);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
