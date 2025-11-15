package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class RR_TrajectoryLineToLinearHeading extends CommandBase {
    private final MecanumDriveSubsystem m_drive;
    private final Pose2d m_endPose;
    private final boolean m_reversed;
    private final double m_velocityOverride;

    public RR_TrajectoryLineToLinearHeading(MecanumDriveSubsystem p_drive, Pose2d p_endPose,
                                            boolean p_reversed, double p_velocityOverride){
        m_drive = p_drive;

        m_endPose = p_endPose;
        m_reversed = p_reversed;
        m_velocityOverride = p_velocityOverride;

        addRequirements(m_drive);
    }

    @Override
    public void initialize(){
        Trajectory trajectory = m_drive.trajectoryBuilder(m_drive.getPoseEstimate(), m_reversed)
                .lineToLinearHeading(
                        m_endPose
                        ,SampleMecanumDrive.getVelocityConstraint(m_velocityOverride, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(m_velocityOverride)
                )
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
