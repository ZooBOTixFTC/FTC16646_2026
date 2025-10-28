package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.CMD_AutoColorSwap;
import org.firstinspires.ftc.teamcode.commands.CMD_Intake;
import org.firstinspires.ftc.teamcode.commands.CMD_IntakeToggle;
import org.firstinspires.ftc.teamcode.commands.CMD_Kick;
import org.firstinspires.ftc.teamcode.commands.CMD_ReadPattern;
import org.firstinspires.ftc.teamcode.commands.CMD_Shoot;
import org.firstinspires.ftc.teamcode.commands.CMD_ShootAll;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryBackwardFromCurrent;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryForwardFromCurrent;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryLineFromCurrent;
import org.firstinspires.ftc.teamcode.commands.RR_TurnCommand;

@Autonomous(name = "Close Wall Blue", preselectTeleOp = "Teleop Blue", group = "Auto Blue")
public class AUTO_CloseWallBlue extends Robot_Auto {


    private Trajectory alignFirstVolley, park;
    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(63,39,Math.toRadians(90)));

        alignFirstVolley = m_robot.drivetrain.trajectoryBuilder(getStartingPose(),true)
                .lineToLinearHeading(new Pose2d(60,12, Math.toRadians(-15)))
                .build();

        park = m_robot.drivetrain.trajectoryBuilder(new Pose2d(alignFirstVolley.end().getX(),alignFirstVolley.end().getY(),alignFirstVolley.end().getHeading()+Math.toRadians(110)), false)
                .lineToLinearHeading(new Pose2d(24, 48, Math.toRadians(180)))
                .build();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        SequentialCommandGroup completeTasks = new SequentialCommandGroup();

        completeTasks.addCommands(
                new InstantCommand(()-> m_robot.m_shooter.setKickerPosition(Constants.ShooterConstants.kKickHome))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain,alignFirstVolley)
                ,new CMD_ReadPattern(m_robot.m_vision)
                ,new CMD_AutoColorSwap(m_robot.m_turntable)
                ,new RR_TurnCommand(m_robot.drivetrain,Math.toRadians(110))
                ,new InstantCommand(()-> m_robot.m_shooter.setTargetVel(25500))
                ,new CMD_ShootAll(m_robot.m_shooter, m_robot.m_turntable, m_robot.GlobalVariables)
                ,new InstantCommand(()-> m_robot.m_shooter.setTargetVel(0))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, park)
        );
        return completeTasks;
    }
}