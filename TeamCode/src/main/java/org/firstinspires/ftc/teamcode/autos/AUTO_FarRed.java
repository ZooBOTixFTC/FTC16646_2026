package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.CMD_Shoot;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryForwardFromCurrent;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryLineFromCurrent;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectorySplineFromCurrent;

@Autonomous(name = "Far Red", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_FarRed extends Robot_Auto {
    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-63,-9.5, Math.toRadians(0)));
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        SequentialCommandGroup completeTasks = new SequentialCommandGroup();

        completeTasks.addCommands(
                new CMD_Shoot(m_robot.m_shooter),
                SecondVolley()
        );

        return completeTasks;
    }

    private SequentialCommandGroup SecondVolley() {
        return new SequentialCommandGroup(
                new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
                ,new RR_TrajectorySplineFromCurrent(m_robot.drivetrain, new Pose2d(-36,-24,Math.toRadians(-90)),0,false)
                ,new RR_TrajectoryLineFromCurrent(m_robot.drivetrain, new Pose2d(-36, -60, Math.toRadians(-90)))
                ,new RR_TrajectorySplineFromCurrent(m_robot.drivetrain, new Pose2d(-63, 0, Math.toRadians(315)), Math.toRadians(315), true)
                ,new CMD_Shoot(m_robot.m_shooter)
        );
    }
}