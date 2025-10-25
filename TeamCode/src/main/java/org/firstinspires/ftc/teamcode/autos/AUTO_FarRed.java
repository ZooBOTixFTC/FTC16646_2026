package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.*;

@Autonomous(name = "Far Red", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_FarRed extends Robot_Auto {
    private Trajectory m_lineUpFirstVolley, m_lineUpCloseSpikeMark, m_collectArtifacts, m_backToLaunchZone, m_leave;
    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-63,-9.5, Math.toRadians(0)));
        m_lineUpFirstVolley = m_robot.drivetrain.trajectoryBuilder(getStartingPose(), false)
                .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(-25)))
                .build();

        m_lineUpCloseSpikeMark = m_robot.drivetrain.trajectoryBuilder(m_lineUpFirstVolley.end(), false)
                .lineToLinearHeading(new Pose2d(-36, -48, Math.toRadians(-90)))
                .build();

        m_collectArtifacts = m_robot.drivetrain.trajectoryBuilder(m_lineUpCloseSpikeMark.end(), false)
                .lineToConstantHeading(new Vector2d(-36, -48))
                .build();

        m_backToLaunchZone = m_robot.drivetrain.trajectoryBuilder(m_collectArtifacts.end(), true)
                .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(155)))
                .build();

        m_leave = m_robot.drivetrain.trajectoryBuilder(m_backToLaunchZone.end(), false)
                .forward(48)
                .build();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        SequentialCommandGroup completeTasks = new SequentialCommandGroup();

        completeTasks.addCommands(
                FirstVolley(),
                CollectArtifacts(),
                SecondVolley()
        );

        return completeTasks;
    }

    private SequentialCommandGroup FirstVolley(){
        return new SequentialCommandGroup(
            new RR_TrajectoryFollowerCommand(m_robot.drivetrain, m_lineUpFirstVolley)
            ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(IntakeConstants.kIntakeOn))
//            ,new CMD_Shoot(m_robot.drivetrain, m_robot.m_shooter, m_robot.m_lift, m_robot.m_intake)
        );
    }

    private SequentialCommandGroup CollectArtifacts(){
        return new SequentialCommandGroup(
            new RR_TrajectoryFollowerCommand(m_robot.drivetrain, m_lineUpCloseSpikeMark)
            ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, m_collectArtifacts)
        );
    }

    private SequentialCommandGroup SecondVolley() {
        return new SequentialCommandGroup(
            new RR_TrajectoryFollowerCommand(m_robot.drivetrain, m_backToLaunchZone)
//            ,new CMD_Shoot(m_robot.m_shooter, m_robot.m_lift, m_robot.m_intake)
            ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(IntakeConstants.kIntakeOff))
            ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, m_leave)
        );
    }
}