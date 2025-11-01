package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ShooterLeft;

public class CMD_GetLeftShooterAtVelocity extends CommandBase {
    private final SUB_ShooterLeft m_shooterLeft;
    public CMD_GetLeftShooterAtVelocity(SUB_ShooterLeft p_shooterLeft) {
        m_shooterLeft = p_shooterLeft;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_shooterLeft.getVel() - m_shooterLeft.getTargetVel()) < ShooterConstants.kTolerance;
    }
}
