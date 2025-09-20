package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class RR_TrajectorySplineFromCurrent extends CommandBase {
    MecanumDriveSubsystem m_drivetrain;
    private final Pose2d m_pose2d;
    private final double m_tangent;
    boolean m_reversed;
    public RR_TrajectorySplineFromCurrent(MecanumDriveSubsystem p_drivetrain, Pose2d p_pose2d,
                                          double p_tangent, boolean p_reversed){
        m_drivetrain = p_drivetrain;
        m_pose2d = p_pose2d;
        m_tangent = p_tangent;
        m_reversed = p_reversed;
    }

    @Override
    public void initialize(){
        Trajectory m_driveForward = m_drivetrain.trajectoryBuilder(m_drivetrain.getPoseEstimate(), m_reversed)
                .splineToLinearHeading(m_pose2d, m_tangent)
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