package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShootRight extends SequentialCommandGroup {
    public CMD_ShootRight(SUB_Shooter p_shooter, GamepadEx m_gamepad){

        addRequirements(p_shooter);
        addCommands(
                new InstantCommand(()-> p_shooter.setVelocity(ShooterConstants.kMaxVelDegPerSec)),
                new CMD_GetShooterAtVelocity(p_shooter),
                new InstantCommand(()-> p_shooter.setFeederPower(ShooterConstants.kFeedPowerOn)),
                new InstantCommand(()-> p_shooter.setKickRightPower(ShooterConstants.kKickOn)),
                new CMD_WaitUntilTrue(m_gamepad,GamepadKeys.Button.RIGHT_BUMPER)
        );
    }
}
