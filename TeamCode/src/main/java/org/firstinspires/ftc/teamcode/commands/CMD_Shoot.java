package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Turntable;

public class CMD_Shoot extends SequentialCommandGroup {
    public CMD_Shoot (SUB_Shooter p_Shooter, SUB_Turntable p_turntable) {
        addCommands(
                new WaitCommand(350)
                ,new CMD_GetShooterAtVelocity(p_Shooter)
                ,new CMD_Kick(p_Shooter)
                ,new WaitCommand(500)
                ,new InstantCommand(p_turntable::rotateRight)
        );
    }
}
