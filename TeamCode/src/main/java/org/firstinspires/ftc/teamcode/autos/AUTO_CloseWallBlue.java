package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.CMD_Intake;
import org.firstinspires.ftc.teamcode.commands.CMD_IntakeToggle;
import org.firstinspires.ftc.teamcode.commands.CMD_Kick;
import org.firstinspires.ftc.teamcode.commands.CMD_Shoot;
import org.firstinspires.ftc.teamcode.commands.CMD_ShootAll;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryBackwardFromCurrent;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryForwardFromCurrent;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryLineFromCurrent;

@Autonomous(name = "Close Wall Blue", preselectTeleOp = "Teleop BLue", group = "Auto Blue")
public class AUTO_CloseWallBlue extends Robot_Auto {


    private Trajectory firstVolley, secondVolley;
    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(41.5,65,Math.toRadians(90)));

        firstVolley = m_robot.drivetrain.trajectoryBuilder(getStartingPose(),false)
                .back(26)
                .build();
        secondVolley = m_robot.drivetrain.trajectoryBuilder(firstVolley.end(), false)
                .strafeLeft(95)
                .build();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        SequentialCommandGroup completeTasks = new SequentialCommandGroup();

        completeTasks.addCommands(
                new CMD_Kick(m_robot.m_shooter, m_robot.m_turntable, m_robot.GlobalVariables)
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain,firstVolley)
                ,new CMD_ShootAll(m_robot.m_shooter,m_robot.m_turntable,m_robot.GlobalVariables)
                ,new InstantCommand(()-> m_robot.m_shooter.setVelocity(0))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, secondVolley)
        );
        return completeTasks;
    }
}