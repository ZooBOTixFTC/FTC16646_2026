package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class RR_TrajectoryLineFromCurrent extends CommandBase {
    MecanumDriveSubsystem m_drivetrain;
    private final Pose2d m_pose2d;
    public RR_TrajectoryLineFromCurrent(MecanumDriveSubsystem p_drivetrain, Pose2d p_pose2d){
        m_drivetrain = p_drivetrain;
        m_pose2d = p_pose2d;
    }

    @Override
    public void initialize(){
        Trajectory m_driveForward = m_drivetrain.trajectoryBuilder(m_drivetrain.getPoseEstimate())
                .lineToLinearHeading(m_pose2d)
                .build();

        m_drivetrain.followTrajectory(m_driveForward);
    }

    @Override
    public void execute(){
        m_drivetrain.update();
    }

    @Override
    public boolean isFinished(){
        return Thread.currentThread().isInterrupted() || !m_drivetrain.isBusy();
    }
}