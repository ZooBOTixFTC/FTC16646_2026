package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_GetRightShooterAtVelocity extends CommandBase {
    private final SUB_Shooter m_shooter;
    public CMD_GetRightShooterAtVelocity(SUB_Shooter p_shooter) {
        m_shooter = p_shooter;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_shooter.getVelocityRight() - m_shooter.getTargetVelRight()) < ShooterConstants.kTolerance;
    }
}
