package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.CMD_AutoColorSwap;
import org.firstinspires.ftc.teamcode.commands.CMD_ReadMotif;
import org.firstinspires.ftc.teamcode.commands.CMD_ShootAll;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.RR_TurnCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "New Close Ramp Red", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_NewCloseRampRed extends Robot_Auto {


    private Trajectory readMotif, lineUpSpikeMark, intakeBallOne, intakeBallTwo, secondVolley, secondIntakeBallOne, secondIntakeBallTwo, thirdVolley, thirdIntakeAllign,thirdIntake,fourthVolley, park;
    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(41.5,-65,Math.toRadians(0)));

        readMotif = m_robot.drivetrain.trajectoryBuilder(getStartingPose(), false)
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(0)))
                .build();

        Pose2d lineUpSpikeMarkStartPose = new Pose2d(readMotif.end().getX(), readMotif.end().getY(),
                readMotif.end().getHeading() + Math.toRadians(-45));

        lineUpSpikeMark = m_robot.drivetrain.trajectoryBuilder(lineUpSpikeMarkStartPose, false)
                .lineToLinearHeading(new Pose2d(7, -33, Math.toRadians(-90)))
                .build();

        intakeBallOne = m_robot.drivetrain.trajectoryBuilder(lineUpSpikeMark.end(), false)
                .forward(6)
                .build();

        intakeBallTwo = m_robot.drivetrain.trajectoryBuilder(intakeBallOne.end(), false)
                .forward(13)
                .build();

        secondVolley = m_robot.drivetrain.trajectoryBuilder(intakeBallTwo.end(), false)
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(-42.5)))
                .build();

        secondIntakeBallOne = m_robot.drivetrain.trajectoryBuilder(secondVolley.end(), false)
                .lineToLinearHeading(new Pose2d(-16, -36, Math.toRadians(-90)))
                .build();
        secondIntakeBallTwo = m_robot.drivetrain.trajectoryBuilder(secondIntakeBallOne.end(), false)
                .forward(6)
                .build();
        thirdVolley = m_robot.drivetrain.trajectoryBuilder(secondIntakeBallTwo.end(),false)
                .lineToLinearHeading(new Pose2d(12,-12, Math.toRadians(-42.5)))
                .build();
        thirdIntakeAllign = m_robot.drivetrain.trajectoryBuilder(secondVolley.end(),false)
                .lineToLinearHeading(new Pose2d(-30,-36,Math.toRadians(-90)))
                .build();
        thirdIntake = m_robot.drivetrain.trajectoryBuilder(thirdIntakeAllign.end(),false)
                .forward(21, SampleMecanumDrive.getVelocityConstraint(20,20,25),SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
        fourthVolley = m_robot.drivetrain.trajectoryBuilder(thirdIntake.end(),false)
                .lineToLinearHeading(new Pose2d(12,-12,Math.toRadians(-42.5)))
                .build();
        park = m_robot.drivetrain.trajectoryBuilder(fourthVolley.end(), false)
                .lineToLinearHeading(new Pose2d(7, -33, Math.toRadians(-90)))
                .build();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        SequentialCommandGroup completeTasks = new SequentialCommandGroup();

        completeTasks.addCommands(
                firstVolley()
                ,intakeSpikeMark()
                ,secondVolley()
                ,thirdVolley()
                ,fourthVolley()
                ,park()
        );
        return completeTasks;
    }

    private SequentialCommandGroup firstVolley(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> m_robot.m_shooter.setKickerPosition(Constants.ShooterConstants.kKickHome))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, readMotif)
                ,new CMD_ReadMotif(m_robot.m_vision)
                ,new CMD_AutoColorSwap(m_robot.m_turntable)
                ,new RR_TurnCommand(m_robot.drivetrain, Math.toRadians(-45))
                ,new CMD_ShootAll(m_robot.m_shooter, m_robot.m_turntable, 27000)
                ,new WaitCommand(250)
                ,new InstantCommand(()-> m_robot.m_shooter.setTargetVel(0))
        );
    }

    private SequentialCommandGroup intakeSpikeMark(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> m_robot.m_turntable.setPos(60))
                ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, lineUpSpikeMark)
                ,new WaitCommand(500)
                ,new InstantCommand(()-> m_robot.m_turntable.setPos(180))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, intakeBallOne)
                ,new WaitCommand(500)
                ,new InstantCommand(()-> m_robot.m_turntable.setPos(300))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, intakeBallTwo)
                ,new WaitCommand(500)
                ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
        );
    }

    private SequentialCommandGroup secondVolley(){
        return new SequentialCommandGroup(
                new InstantCommand(()->m_robot.m_turntable.setPos(120))
                ,new WaitCommand(200)
                ,new CMD_AutoColorSwap(m_robot.m_turntable)
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, secondVolley)
                ,new CMD_ShootAll(m_robot.m_shooter, m_robot.m_turntable, 27000)
                ,new WaitCommand(250)
                ,new InstantCommand(()-> m_robot.m_shooter.setTargetVel(0))
        );
    }

    private SequentialCommandGroup thirdVolley(){
        return new SequentialCommandGroup(
                new InstantCommand(()->m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, secondIntakeBallOne)
                ,new InstantCommand(()->m_robot.m_turntable.rotateRight())
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, secondIntakeBallTwo)
                ,new InstantCommand(()->m_robot.m_turntable.rotateRight())
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, park)
                ,new InstantCommand(()->m_robot.m_turntable.rotateRight())
                ,new InstantCommand(()->m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
        );
    }
    private SequentialCommandGroup fourthVolley(){
        return new SequentialCommandGroup(
                new RR_TrajectoryFollowerCommand(m_robot.drivetrain,thirdIntakeAllign)
                ,new InstantCommand(()->m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain,thirdIntake)
                ,new InstantCommand(()->m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain,fourthVolley)
                ,new CMD_ShootAll(m_robot.m_shooter, m_robot.m_turntable)
        );
    }
    private SequentialCommandGroup park(){
        return new SequentialCommandGroup(
                new InstantCommand(()->m_robot.m_shooter.setTargetVel(0))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain,park)
        );
    }
}