package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShootLeft extends SequentialCommandGroup {
    public CMD_ShootLeft(SUB_Shooter p_shooter){
        addRequirements(p_shooter);
        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(()-> p_shooter.setTargetVelLeft(ShooterConstants.kMaxVelDegPerSec)),
                new CMD_GetLeftShooterAtVelocity(p_shooter),
                new CMD_KickLeft(p_shooter)
            )
        );
    }
}
