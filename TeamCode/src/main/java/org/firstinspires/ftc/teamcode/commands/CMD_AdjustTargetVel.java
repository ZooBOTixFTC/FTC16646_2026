package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ShooterLeft;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ShooterRight;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Vision;

public class CMD_AdjustTargetVel extends CommandBase {
    private final SUB_ShooterLeft m_shooterLeft;
    private final SUB_ShooterRight m_shooterRight;
    private final SUB_Vision m_vision;
    public CMD_AdjustTargetVel(SUB_ShooterLeft p_shooterLeft, SUB_ShooterRight p_shooterRight,  SUB_Vision p_vision){
        m_shooterLeft = p_shooterLeft;
        m_shooterRight = p_shooterRight;
        m_vision = p_vision;
    }

    @Override
    public void initialize(){
        if(m_vision.getDistToTag() < 90 && !m_vision.getDetections().isEmpty()){
            m_shooterLeft.setGoal(Constants.ShooterConstants.kMidFieldVel);
            m_shooterRight.setGoal(Constants.ShooterConstants.kMidFieldVel);
        }else{
            m_shooterLeft.setGoal(Constants.ShooterConstants.kMaxVel);
            m_shooterRight.setGoal(Constants.ShooterConstants.kMaxVel);
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
