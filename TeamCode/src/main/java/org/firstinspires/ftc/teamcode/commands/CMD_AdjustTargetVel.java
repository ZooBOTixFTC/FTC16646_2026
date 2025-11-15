package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_AdjustTargetVel extends CommandBase {
    private final SUB_Shooter m_shooter;

    public CMD_AdjustTargetVel(SUB_Shooter p_shooter){
        m_shooter = p_shooter;
    }

    @Override
    public void initialize(){
        if(GlobalVariables.m_distToTag < Constants.AutoAlignConstants.kMidRangeThreshold) {
            m_shooter.setShootingVelocity(Constants.ShooterConstants.kCloseVel);
        }else if(GlobalVariables.m_distToTag < Constants.AutoAlignConstants.kDistanceThreshold){
            m_shooter.setShootingVelocity(Constants.ShooterConstants.kMidFieldVel);
        }else{
            m_shooter.setShootingVelocity(Constants.ShooterConstants.kFarVel );
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
