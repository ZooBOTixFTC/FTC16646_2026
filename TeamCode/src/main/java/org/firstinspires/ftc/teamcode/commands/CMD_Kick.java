package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;


public class CMD_Kick extends CommandBase {
    private final ElapsedTime timer = new ElapsedTime();
    private final SUB_Shooter m_shooter;

    public CMD_Kick(SUB_Shooter p_shooter) {
        m_shooter = p_shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setKickerPosition(ShooterConstants.kKickPosition);
        timer.reset();
    }
    @Override
    public boolean isFinished() {
        return timer.seconds() >= 0.25;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setKickerPosition(0);
    }
}