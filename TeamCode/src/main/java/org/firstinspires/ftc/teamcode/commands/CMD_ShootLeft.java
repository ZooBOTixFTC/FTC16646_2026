package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShootLeft extends SequentialCommandGroup {
    public CMD_ShootLeft(SUB_Shooter p_shooter){

        addRequirements(p_shooter);
        addCommands(
            new InstantCommand(()-> p_shooter.setVelocity(1)),
            new WaitCommand(2000),
//            new CMD_GetShooterAtVelocity(p_shooter),
            new CMD_KickLeft(p_shooter)
        );
    }
}
