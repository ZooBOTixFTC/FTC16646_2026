package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShootPattern extends SequentialCommandGroup {
    public CMD_ShootPattern(SUB_Shooter p_shooter, boolean p_leftChamber){

        addRequirements(p_shooter);
        addCommands(
            new InstantCommand(()-> p_shooter.setVelocity(0.0)),
            new CMD_GetShooterAtVelocity(p_shooter),
            new ConditionalCommand(
                new CMD_KickLeft(p_shooter),
                new CMD_KickRight(p_shooter),
                ()-> p_leftChamber
            )

        );
    }
}
