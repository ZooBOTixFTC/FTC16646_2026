package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.ColorConstants.ColorEnum;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_Shoot extends SequentialCommandGroup {
    public CMD_Shoot(SUB_Shooter p_shooter){
        addRequirements(p_shooter);
        addCommands(
                //set initial target velocity and wait until that speed has been reached
                new InstantCommand(()-> p_shooter.setVelocity(Constants.ShooterConstants.kMaxVelDegPerSec))
                ,new CMD_GetShooterAtVelocity(p_shooter)
                //after we reach target velocity spin the left kick wheel
                ,new CMD_KickLeft(p_shooter)
                //wait for velocity to come back up and spin the right kick wheel
                ,new CMD_GetShooterAtVelocity(p_shooter)
                ,new CMD_KickRight(p_shooter)
                //after velocity comes back up again, spin both kick wheels to shoot the last ball
                ,new CMD_GetShooterAtVelocity(p_shooter)
                ,new CMD_KickLeft(p_shooter)
                ,new CMD_KickRight(p_shooter)
        );
    }
}
