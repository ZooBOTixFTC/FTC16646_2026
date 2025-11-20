package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShootInterrupt extends CommandBase {
    public CMD_ShootInterrupt(MecanumDriveSubsystem p_drive, SUB_Shooter p_shooter){
        addRequirements(p_drive, p_shooter);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
