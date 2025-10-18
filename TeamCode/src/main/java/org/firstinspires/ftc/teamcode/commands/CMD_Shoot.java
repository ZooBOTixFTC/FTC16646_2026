package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_Shoot extends SequentialCommandGroup {
    public CMD_Shoot (SUB_Shooter p_Shooter) {
        addCommands(
                new InstantCommand(()->p_Shooter.setVelocity(Constants.ShooterConstants.kMaxVelDegPerSec))
                ,new CMD_GetShooterAtVelocity(p_Shooter)
                ,new CMD_Kick(p_Shooter)
        );
    }
}
