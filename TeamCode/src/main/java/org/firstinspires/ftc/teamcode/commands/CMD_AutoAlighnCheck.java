package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Vision;

public class CMD_AutoAlighnCheck extends CommandBase {
    private final MecanumDriveSubsystem m_drive;
    private final SUB_Vision m_vision;
    private boolean isFinished;
    public CMD_AutoAlighnCheck(MecanumDriveSubsystem p_drive, SUB_Vision p_vision){
        m_drive = p_drive;
        m_vision = p_vision;

        addRequirements(m_vision);
    }

    @Override
    public void initialize(){

        if (GlobalVariables.m_distToTag > 105){
            new CMD_AlignTarget(m_drive, m_vision).schedule();
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}