package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryFollowerCommand;

public class AUTO_Test extends Robot_Auto {

    private Trajectory m_testTrajectory;

    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-41, -63.625, Math.toRadians(180)));

        m_testTrajectory = m_robot.drivetrain.trajectoryBuilder(getStartingPose(), false)
                .splineTo(new Vector2d(0, 0), Math.toRadians(-180))
                .build();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        SequentialCommandGroup completeTasks = new SequentialCommandGroup();

        completeTasks.addCommands(
            new RR_TrajectoryFollowerCommand(m_robot.drivetrain, m_testTrajectory)
        );

        return completeTasks;
    }
}
