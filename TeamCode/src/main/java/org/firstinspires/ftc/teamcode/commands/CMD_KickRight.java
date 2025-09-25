package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;


public class CMD_KickRight extends CommandBase {
    private final ElapsedTime timer = new ElapsedTime();
    private final SUB_Shooter m_shooter;

    public CMD_KickRight(SUB_Shooter p_Shooter) {
        m_shooter = p_Shooter;
    }

    @Override
    public void initialize() {
        m_shooter.setKickRightPower(Constants.ShooterConstants.kKickOn);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= Constants.ShooterConstants.kickDuration;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setKickRightPower(Constants.ShooterConstants.kKickOff);
    }
}
