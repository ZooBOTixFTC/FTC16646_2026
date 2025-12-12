package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Limelight;

public class CMD_AutoAlignCheck extends CommandBase {
    private final MecanumDriveSubsystem m_drive;
    private final SUB_Limelight m_limelight;
    public CMD_AutoAlignCheck(MecanumDriveSubsystem p_drive, SUB_Limelight p_limelight){
        m_drive = p_drive;
        m_limelight = p_limelight;

        addRequirements(m_limelight);
    }

    @Override
    public void initialize(){
        //if shooting from far launch zone and camera has a tag in view, schedule auto align
        if (GlobalVariables.m_distToTag > Constants.AutoAlignConstants.kDistanceThreshold &&
                !m_limelight.getLatestResult().getFiducialResults().isEmpty()){
            new CMD_AlignTarget(m_drive, m_limelight).schedule();
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
