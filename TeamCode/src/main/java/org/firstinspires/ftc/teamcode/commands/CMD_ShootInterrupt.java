package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Intake;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Limelight;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShootInterrupt extends CommandBase {
    public CMD_ShootInterrupt(MecanumDriveSubsystem p_drive, SUB_Shooter p_shooter, SUB_Limelight p_vision, SUB_Intake p_intake){
        addRequirements(p_drive, p_shooter, p_vision, p_intake);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
