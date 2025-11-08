package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class CMD_DriveInterrupt extends CommandBase {
    private final MecanumDriveSubsystem m_drive;
    public CMD_DriveInterrupt(MecanumDriveSubsystem p_drive){
        m_drive = p_drive;
        addRequirements(p_drive);
    }

    @Override
    public void initialize(){
        m_drive.stop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
