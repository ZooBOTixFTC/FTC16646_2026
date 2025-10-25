package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Auto;

@Autonomous(name = "Far Blue", preselectTeleOp = "Teleop Blue", group = "Auto Blue")
public class AUTO_FarBlue extends Robot_Auto {
    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-63,0,Math.toRadians(45)));
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        SequentialCommandGroup completeTasks = new SequentialCommandGroup();

        completeTasks.addCommands(
                SecondVolley()
        );

        return completeTasks;
    }

    private SequentialCommandGroup SecondVolley() {
        return new SequentialCommandGroup(
                new InstantCommand(()->m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn)),
                new InstantCommand(()->m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
        );
    }
}