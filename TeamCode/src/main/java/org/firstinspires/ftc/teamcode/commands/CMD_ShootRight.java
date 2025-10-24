package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShootRight extends SequentialCommandGroup {
    public CMD_ShootRight(SUB_Shooter p_shooter){
        addRequirements(p_shooter);
        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(()-> p_shooter.setTargetVelRight(ShooterConstants.kMaxVelDegPerSec)),
                new CMD_GetLeftShooterAtVelocity(p_shooter),
                new CMD_KickRight(p_shooter)
            )
        );
    }
}
