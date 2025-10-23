package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Turntable;

public class CMD_Shoot extends SequentialCommandGroup {
    public CMD_Shoot (SUB_Shooter p_Shooter, SUB_Turntable p_turntable, GlobalVariables p_variables) {
        addCommands(
                new InstantCommand(()->p_Shooter.setVelocity(Constants.ShooterConstants.kMaxVelDegPerSec))
                ,new WaitCommand(2000)
//                ,new CMD_GetShooterAtVelocity(p_Shooter)
                ,new CMD_Kick(p_Shooter,p_turntable,p_variables)
        );
    }
}
