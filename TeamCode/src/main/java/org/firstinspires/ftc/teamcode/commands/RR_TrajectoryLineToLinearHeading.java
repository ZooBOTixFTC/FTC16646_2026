package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class RR_TrajectoryLineToLinearHeading extends CommandBase {
    private final MecanumDriveSubsystem m_drive;
    private final Pose2d m_endPose;
    private boolean m_reversed;

    public RR_TrajectoryLineToLinearHeading(MecanumDriveSubsystem p_drive, Pose2d p_endPose, boolean p_reversed){
        m_drive = p_drive;

        m_endPose = p_endPose;
        m_reversed = p_reversed;

        addRequirements(m_drive);
    }

    @Override
    public void initialize(){
        Trajectory trajectory = m_drive.trajectoryBuilder(m_drive.getPoseEstimate(), m_reversed)
                .lineToLinearHeading(m_endPose)
                .build();

        m_drive.followTrajectory(trajectory);
    }

    @Override
    public void execute(){
        m_drive.update();
    }

    @Override
    public boolean isFinished(){
        return Thread.currentThread().isInterrupted() || !m_drive.isBusy();
    }
}
