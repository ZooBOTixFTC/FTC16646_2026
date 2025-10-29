package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Turntable;

public class CMD_Shoot extends SequentialCommandGroup {
    public CMD_Shoot (SUB_Shooter p_shooter, SUB_Turntable p_turntable) {
        addCommands(
                new CMD_GetShooterAtVelocity(p_shooter)
                ,new CMD_Kick(p_shooter, p_turntable)
                ,new InstantCommand(p_turntable::rotateRight)
        );
    }
}
