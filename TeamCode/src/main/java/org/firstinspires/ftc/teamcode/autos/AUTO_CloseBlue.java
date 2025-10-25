package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Auto;

@Autonomous(name = "Close Blue", preselectTeleOp = "Teleop Blue", group = "Auto Blue")
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
        );
    }

    private SequentialCommandGroup secondVolley() {
        return new SequentialCommandGroup(
        );
    }
}