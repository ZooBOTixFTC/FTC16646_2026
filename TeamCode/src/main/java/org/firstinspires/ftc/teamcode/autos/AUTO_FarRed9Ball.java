package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.*;

@Autonomous(name = "Far Red 9 Ball", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_FarRed9Ball extends Robot_Auto {
    private Trajectory moveAway;

    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-64.75, -15.5, Math.toRadians(0)));
        GlobalVariables.m_far = true;
        GlobalVariables.m_red = true;

        moveAway = m_robot.drivetrain.trajectoryBuilder(getStartingPose(), false)
                .lineToLinearHeading(new Pose2d(-58, -15, Math.toRadians(-17.5)))
                .build();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        return new SequentialCommandGroup(
                new RR_TrajectoryFollowerCommand(m_robot.drivetrain, moveAway)
                ,volley()
                ,intakeFirstSpikeMark()
                ,volley()
                ,intakeCorner()
                ,volley()
                //run to corner one last time to grab any remaining balls for tele
                ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-64, -62, Math.toRadians(-90)), false)
        );
    }

    private SequentialCommandGroup volley(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kPreRev))
                ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
                ,new CMD_AlignTarget(m_robot.drivetrain, m_robot.m_vision)
                ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kFarVel))
                ,new CMD_ShootAuto(m_robot.m_shooter, m_robot.m_kicker, m_robot.m_intake)
                ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(0))
        );
    }

    private SequentialCommandGroup intakeFirstSpikeMark(){
        return new SequentialCommandGroup(
                new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-36, -30, Math.toRadians(-90)), false)
                ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-36, -62, Math.toRadians(-90)), false)
                ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-60, -15, Math.toRadians(-17.5)), false)
        );
    }

    private SequentialCommandGroup intakeCorner(){
        return new SequentialCommandGroup(
                new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-64, -62, Math.toRadians(-90)), false)
                ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-60, -15, Math.toRadians(-17.5)), false)
        );
    }
}
