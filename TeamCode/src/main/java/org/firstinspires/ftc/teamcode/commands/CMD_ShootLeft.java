package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Lift;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShootLeft extends SequentialCommandGroup {
    public CMD_ShootLeft(SUB_Shooter p_shooter, SUB_Lift p_lift){
        addRequirements(p_shooter);
        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(()-> p_shooter.setTargetVelLeft(ShooterConstants.kMaxVelDegPerSec)),
//                new CMD_GetLeftShooterAtVelocity(p_shooter),
                new WaitCommand(2000),
                new CMD_Kick(p_lift)
            )
        );
    }
}
