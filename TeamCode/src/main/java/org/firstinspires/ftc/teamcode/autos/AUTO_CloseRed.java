package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.CMD_ShootAuto;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryFollowerCommand;

@Autonomous(name = "Close Red", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_CloseRed extends Robot_Auto {
    private Trajectory firstVolley,  lineupFirstSpikeMark, intakeFirstSpikeMark, secondVolley,
            lineupSecondSpikeMark, intakeSecondSpikeMark, thirdVolley, park;

    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(50, -47, Math.toRadians(-50)));
        GlobalVariables.m_far = false;
        GlobalVariables.m_red = true;

        firstVolley = m_robot.drivetrain.trajectoryBuilder(getStartingPose(), true)
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(-41)))
                .build();
        
        lineupFirstSpikeMark = m_robot.drivetrain.trajectoryBuilder(firstVolley.end(), false)
                .lineToLinearHeading(new Pose2d(6, -24,  Math.toRadians(-90)))
                .build();
        
        intakeFirstSpikeMark = m_robot.drivetrain.trajectoryBuilder(lineupFirstSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(6, -52, Math.toRadians(-90)))
                .build();
        
        secondVolley = m_robot.drivetrain.trajectoryBuilder(intakeFirstSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(-41)))
                .build();

        lineupSecondSpikeMark = m_robot.drivetrain.trajectoryBuilder(secondVolley.end(), false)
                .lineToLinearHeading(new Pose2d(-12, -24,  Math.toRadians(-90)))
                .build();

        intakeSecondSpikeMark = m_robot.drivetrain.trajectoryBuilder(lineupSecondSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(-12, -58, Math.toRadians(-90)))
                .build();

        thirdVolley = m_robot.drivetrain.trajectoryBuilder(intakeSecondSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(-41)))
                .build();

        park = m_robot.drivetrain.trajectoryBuilder(thirdVolley.end(), false)
                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(-90)))
                .build();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        return new SequentialCommandGroup(
            new RR_TrajectoryFollowerCommand(m_robot.drivetrain, firstVolley)
            ,volley()
            ,intakeFirstSpikeMark()
            ,volley()
            ,intakeSecondSpikeMark()
            ,volley()
            //
            ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, park)
        );
    }

    private SequentialCommandGroup volley(){
        return new SequentialCommandGroup(
            new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kMidFieldVel))
            ,new CMD_ShootAuto(m_robot.m_shooter, m_robot.m_kicker, m_robot.m_intake)
            ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(0))
        );
    }
    
    private SequentialCommandGroup intakeFirstSpikeMark(){
        return new SequentialCommandGroup(
            new RR_TrajectoryFollowerCommand(m_robot.drivetrain, lineupFirstSpikeMark)
            ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, intakeFirstSpikeMark)
            ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, secondVolley)
        );
    }

    private SequentialCommandGroup intakeSecondSpikeMark(){
        return new SequentialCommandGroup(
                new RR_TrajectoryFollowerCommand(m_robot.drivetrain, lineupSecondSpikeMark)
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, intakeSecondSpikeMark)
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, thirdVolley)
        );
    }
}
