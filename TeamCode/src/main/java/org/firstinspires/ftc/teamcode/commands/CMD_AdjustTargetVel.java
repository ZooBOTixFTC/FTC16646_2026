package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Vision;

public class CMD_AdjustTargetVel extends CommandBase {
    private final SUB_Shooter m_shooter;
    private final SUB_Vision m_vision;
    public CMD_AdjustTargetVel(SUB_Shooter p_shooter, SUB_Vision p_vision){
        m_shooter = p_shooter;
        m_vision = p_vision;
    }

    @Override
    public void initialize(){
        if(m_vision.getDistToTag() < 90 && !m_vision.getDetections().isEmpty()){
            m_shooter.setTargetVel(Constants.ShooterConstants.kMidFieldVel);
        }else{
            m_shooter.setTargetVel(Constants.ShooterConstants.kMaxVelDegPerSec);
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
