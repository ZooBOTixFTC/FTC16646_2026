package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ShooterLeft;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ShooterRight;

public class CMD_GetShooterAtVelocity extends CommandBase {
    private final SUB_ShooterLeft m_shooterLeft;
    private final SUB_ShooterRight m_shooterRight;
    public CMD_GetShooterAtVelocity(SUB_ShooterLeft p_shooterLeft, SUB_ShooterRight p_shooterRight) {
        m_shooterLeft = p_shooterLeft;
        m_shooterRight = p_shooterRight;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_shooterLeft.getVel() - m_shooterLeft.getTargetVel()) < ShooterConstants.kTolerance
                && (Math.abs(m_shooterRight.getVel()) - m_shooterRight.getTargetVel()) < ShooterConstants.kTolerance;
    }
}
