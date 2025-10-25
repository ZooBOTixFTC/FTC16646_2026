package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Lift;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_Shoot extends SequentialCommandGroup {
    public CMD_Shoot(SUB_Shooter p_shooter, SUB_Lift p_lift){
        addRequirements(p_shooter);
        addCommands(
                //set initial target velocity and wait until that speed has been reached
                new InstantCommand(()-> p_shooter.setTargetVelRight(Constants.ShooterConstants.kMaxVelDegPerSec))
                ,new InstantCommand(()-> p_shooter.setTargetVelRight(Constants.ShooterConstants.kMaxVelDegPerSec))
//                ,new CMD_GetLeftShooterAtVelocity(p_shooter)
//                ,new CMD_GetRightShooterAtVelocity(p_shooter)
                ,new WaitCommand(2000)
                //after we reach target velocity, kick two artifacts into either shooter ramp
                ,new CMD_Kick(p_lift)
                ,new WaitCommand(1000)
                //wait for velocity to come back up, and kick the remaining artifact
                ,new WaitCommand(2000)
//                ,new CMD_GetLeftShooterAtVelocity(p_shooter)
//                ,new CMD_GetRightShooterAtVelocity(p_shooter)
                ,new CMD_Kick(p_lift)
        );
    }
}
