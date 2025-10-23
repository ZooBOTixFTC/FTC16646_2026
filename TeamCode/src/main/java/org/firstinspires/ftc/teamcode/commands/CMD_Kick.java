package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Turntable;


public class CMD_Kick extends CommandBase {
    private final ElapsedTime timer = new ElapsedTime();
    private final SUB_Shooter m_shooter;
    private final SUB_Turntable m_turntable;
    private final GlobalVariables m_variables;
    boolean isFinished;

    public CMD_Kick(SUB_Shooter p_shooter, SUB_Turntable p_turntable, GlobalVariables p_variables) {
        m_shooter = p_shooter;
        m_turntable = p_turntable;
        m_variables = p_variables;
    }

    @Override
    public void initialize() {
        timer.reset();
//        if(m_turntable.getPos() - m_turntable.getT)
        m_shooter.setKickerPosition(ShooterConstants.kKickPosition);
        m_variables.setKickerHomed(false);
    }
    @Override
    public boolean isFinished() {
        return isFinished || timer.seconds() >= ShooterConstants.kKickDuration;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setKickerPosition(ShooterConstants.kKickHome);
        m_variables.setKickerHomed(true);
    }
}