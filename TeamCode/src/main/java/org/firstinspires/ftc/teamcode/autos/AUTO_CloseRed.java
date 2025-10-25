package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.CMD_Intake;
import org.firstinspires.ftc.teamcode.commands.CMD_IntakeToggle;
import org.firstinspires.ftc.teamcode.commands.CMD_Shoot;
import org.firstinspires.ftc.teamcode.commands.CMD_ShootAll;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryBackwardFromCurrent;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryForwardFromCurrent;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryLineFromCurrent;

@Autonomous(name = "Close Red", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_CloseRed extends Robot_Auto {

    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-38.21,-38.21,Math.toRadians(-45)));
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        SequentialCommandGroup completeTasks = new SequentialCommandGroup();

        completeTasks.addCommands(
                firstVolley(),
                secondVolley()
        );
        return completeTasks;
    }

    private SequentialCommandGroup firstVolley() {
        return new SequentialCommandGroup(
                new RR_TrajectoryBackwardFromCurrent(m_robot.drivetrain, 25.45, false)
                ,new CMD_ShootAll(m_robot.m_shooter,m_robot.m_turntable,m_robot.GlobalVariables)
        );
    }

    private SequentialCommandGroup secondVolley() {
        return new SequentialCommandGroup(
                new RR_TrajectoryLineFromCurrent(m_robot.drivetrain, new Pose2d(12,-24,-90))
                ,new CMD_IntakeToggle(m_robot.m_intake,m_robot.m_turntable)
                ,new RR_TrajectoryForwardFromCurrent(m_robot.drivetrain,36,false)
                ,new CMD_IntakeToggle(m_robot.m_intake,m_robot.m_turntable)
                ,new RR_TrajectoryLineFromCurrent(m_robot.drivetrain,new Pose2d(-24,24,-45))
                ,new CMD_ShootAll(m_robot.m_shooter,m_robot.m_turntable,m_robot.GlobalVariables)
        );
    }
}