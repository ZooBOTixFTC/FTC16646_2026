package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryForwardFromCurrent;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectorySplineFromCurrent;

@Autonomous(name = "Far Red", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_FarRed extends Robot_Auto {
    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-63,0,Math.toRadians(315)));
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        SequentialCommandGroup completeTasks = new SequentialCommandGroup();

        completeTasks.addCommands(
//                new CMD_Shoot(m_robot.m_shooter,m_robot.m_colorSensor),
                SecondVolley()
        );

        return completeTasks;
    }

    private SequentialCommandGroup SecondVolley() {
        return new SequentialCommandGroup(
                new RR_TrajectorySplineFromCurrent(m_robot.drivetrain, new Pose2d(-36,-24,Math.toRadians(0)),0,false),
                new InstantCommand(()->m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn)),
                new RR_TrajectoryForwardFromCurrent(m_robot.drivetrain,30,false),
                new InstantCommand(()->m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff)),
                new RR_TrajectorySplineFromCurrent(m_robot.drivetrain, new Pose2d(-63,0,Math.toRadians(135)),135,true)
//                ,new CMD_Shoot(m_robot.m_shooter,m_robot.m_colorSensor)
        );
    }
}