package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.CMD_Shoot;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryBackwardFromCurrent;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryForwardFromCurrent;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryLineFromCurrent;

@Autonomous(name = "Close Red", preselectTeleOp = "TeleopRed", group = "Auto Red")
public class AUTO_CloseBlue extends Robot_Auto {

    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(48,48,Math.toRadians(45)));
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        SequentialCommandGroup completeTasks = new SequentialCommandGroup();

        completeTasks.addCommands(
            firstVolley()
            ,secondVolley()
        );
        return completeTasks;
    }

    private SequentialCommandGroup firstVolley() {
        return new SequentialCommandGroup(
                new RR_TrajectoryBackwardFromCurrent(m_robot.drivetrain, 25.45, false),
                new CMD_Shoot(m_robot.m_shooter,m_robot.m_colorSensor)
        );
    }

    private SequentialCommandGroup secondVolley() {
        return new SequentialCommandGroup(
                new RR_TrajectoryLineFromCurrent(m_robot.drivetrain, new Pose2d(12,24,90)),
                new InstantCommand(()->m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn)),
                new RR_TrajectoryForwardFromCurrent(m_robot.drivetrain,30,false),
                new InstantCommand(()->m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff)),
                new RR_TrajectoryBackwardFromCurrent(m_robot.drivetrain,56,false),
                new CMD_Shoot(m_robot.m_shooter,m_robot.m_colorSensor)
        );
    }
}