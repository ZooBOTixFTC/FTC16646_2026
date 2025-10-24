package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_GetLeftShooterAtVelocity extends CommandBase {
    private final SUB_Shooter m_shooter;
    public CMD_GetLeftShooterAtVelocity(SUB_Shooter p_shooter) {
        m_shooter = p_shooter;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_shooter.getVelocityLeft() - m_shooter.getTargetVelLeft()) < ShooterConstants.kTolerance;
    }
}
