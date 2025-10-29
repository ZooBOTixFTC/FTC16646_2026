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

    public CMD_Kick(SUB_Shooter p_shooter, SUB_Turntable p_turntable) {
        m_shooter = p_shooter;
        m_turntable = p_turntable;
    }

    @Override
    public void initialize() {
        if(m_turntable.getVelo() > 3 || GlobalVariables.m_intakeReverse || GlobalVariables.m_intakeOn) return;
        GlobalVariables.m_kickerHomed = false;
        timer.reset();
        m_shooter.setKickerPosition(ShooterConstants.kKickPosition);
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= ShooterConstants.kKickDuration;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setKickerPosition(ShooterConstants.kKickHome);
        GlobalVariables.m_kickerHomed = false;
    }
}