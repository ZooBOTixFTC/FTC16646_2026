package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ShooterRight;

public class CMD_GetRightShooterAtVelocity extends CommandBase {
    private final SUB_ShooterRight m_shooterRight;
    public CMD_GetRightShooterAtVelocity(SUB_ShooterRight p_shooterRight) {
        m_shooterRight = p_shooterRight;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_shooterRight.getVel() - m_shooterRight.getTargetVel()) < ShooterConstants.kTolerance;
    }
}
