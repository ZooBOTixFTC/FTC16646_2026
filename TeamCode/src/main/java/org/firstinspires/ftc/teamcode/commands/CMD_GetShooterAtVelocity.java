package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_GetShooterAtVelocity extends CommandBase {
    private final SUB_Shooter m_shooter;

    private int atVelCounter;

    public CMD_GetShooterAtVelocity(SUB_Shooter p_shooter){
        m_shooter = p_shooter;
    }

    @Override
    public void initialize(){
        atVelCounter = 0;
    }

    @Override
    public void execute(){
        if(m_shooter.isReadyToLaunch()){
            atVelCounter++;
        }else{
            atVelCounter = 0;
        }
    }

    @Override
    public boolean isFinished(){
        return atVelCounter >= 10;
    }
}
