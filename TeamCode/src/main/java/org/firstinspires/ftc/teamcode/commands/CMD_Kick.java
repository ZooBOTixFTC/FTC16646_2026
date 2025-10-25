package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.LiftConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Lift;


public class CMD_Kick extends CommandBase {
    private final ElapsedTime timer = new ElapsedTime();
    private final SUB_Lift m_lift;

    public CMD_Kick(SUB_Lift p_lift) {
        m_lift = p_lift;
        addRequirements(m_lift);
    }

    @Override
    public void initialize() {
        m_lift.setTargetPos(LiftConstants.kKick);
        timer.reset();
    }
    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= LiftConstants.kKickDuration;
    }

    @Override
    public void end(boolean interrupted) {
        m_lift.setTargetPos(LiftConstants.kHome);
    }
}
