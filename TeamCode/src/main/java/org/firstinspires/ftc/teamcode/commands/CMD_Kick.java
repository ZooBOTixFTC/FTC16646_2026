package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.LiftConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Lift;


public class CMD_Kick extends CommandBase {
    private final ElapsedTime timer = new ElapsedTime();
    private final SUB_Lift m_lift;

    private boolean inPos;

    public CMD_Kick(SUB_Lift p_lift) {
        m_lift = p_lift;
        addRequirements(m_lift);
    }

    @Override
    public void initialize() {
        m_lift.setTargetPos(LiftConstants.kKick);
        timer.reset();

        inPos = false;
    }

    @Override
    public void execute(){
        if(Math.abs(m_lift.getPos() - m_lift.getTargetPos()) < 2 && !inPos){
            m_lift.setTargetPos(LiftConstants.kHome);
            inPos = true;
        }
    }

    @Override
    public boolean isFinished() {
        //finish after reaching target pos and having kicked
        return inPos && Math.abs(m_lift.getPos() - m_lift.getTargetPos()) < 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        m_lift.setTargetPos(LiftConstants.kHome);
    }
}
