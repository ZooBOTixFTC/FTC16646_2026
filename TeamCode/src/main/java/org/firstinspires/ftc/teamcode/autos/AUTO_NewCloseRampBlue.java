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
import org.firstinspires.ftc.teamcode.commands.CMD_IntakeToggle;
import org.firstinspires.ftc.teamcode.commands.CMD_ReadMotif;
import org.firstinspires.ftc.teamcode.commands.CMD_ShootAll;
import org.firstinspires.ftc.teamcode.commands.RR_TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.RR_TurnCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "New Close Ramp Blue", preselectTeleOp = "Teleop Blue", group = "Auto Blue")
public class AUTO_NewCloseRampBlue extends Robot_Auto {


    private Trajectory readMotif, lineUpSpikeMark, intakeBallOne, intakeBallTwo, secondVolleyTraj, secondIntakeAllign, secondIntakeBallOne, secondIntakeBallTwo, thirdVolleyTraj, thirdIntakeAllign,thirdIntake, fourthVolleyTraj, park;
    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(41.5,65,Math.toRadians(0)));

        readMotif = m_robot.drivetrain.trajectoryBuilder(getStartingPose(), false)
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(0)))
                .build();

        Pose2d lineUpSpikeMarkStartPose = new Pose2d(readMotif.end().getX(), readMotif.end().getY(),
                readMotif.end().getHeading() + Math.toRadians(45));

        lineUpSpikeMark = m_robot.drivetrain.trajectoryBuilder(lineUpSpikeMarkStartPose, false)
                .lineToLinearHeading(new Pose2d(7, 33, Math.toRadians(90)))
                .build();

        intakeBallOne = m_robot.drivetrain.trajectoryBuilder(lineUpSpikeMark.end(), false)
                .forward(6)
                .build();

        intakeBallTwo = m_robot.drivetrain.trajectoryBuilder(intakeBallOne.end(), false)
                .forward(13)
                .build();

        secondVolleyTraj = m_robot.drivetrain.trajectoryBuilder(intakeBallTwo.end(), false)
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(42.5)))
                .build();
        secondIntakeAllign = m_robot.drivetrain.trajectoryBuilder(secondVolleyTraj.end(), false)
                .lineToLinearHeading(new Pose2d(-16,36, Math.toRadians(90)))
                .build();

        secondIntakeBallOne = m_robot.drivetrain.trajectoryBuilder(secondIntakeAllign.end(), false)
                .forward(6)
                .build();
        secondIntakeBallTwo = m_robot.drivetrain.trajectoryBuilder(secondIntakeBallOne.end(), false)
                .forward(13)
                .build();
        thirdVolleyTraj = m_robot.drivetrain.trajectoryBuilder(secondIntakeBallTwo.end(),false)
                .lineToLinearHeading(new Pose2d(12,12, Math.toRadians(42.5)))
                .build();
        thirdIntakeAllign = m_robot.drivetrain.trajectoryBuilder(thirdVolleyTraj.end(),false)
                .lineToLinearHeading(new Pose2d(-30,36,Math.toRadians(90)))
                .build();
        thirdIntake = m_robot.drivetrain.trajectoryBuilder(thirdIntakeAllign.end(),false)
                .forward(21, SampleMecanumDrive.getVelocityConstraint(20,20,25),SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
        fourthVolleyTraj = m_robot.drivetrain.trajectoryBuilder(thirdIntake.end(),false)
                .lineToLinearHeading(new Pose2d(12,12,Math.toRadians(42.5)))
                .build();
        park = m_robot.drivetrain.trajectoryBuilder(fourthVolleyTraj.end(), false)
                .lineToLinearHeading(new Pose2d(7, 33, Math.toRadians(90)))
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
                ,new RR_TurnCommand(m_robot.drivetrain, Math.toRadians(45))
                ,new CMD_ShootAll(m_robot.m_shooter, m_robot.m_turntable, 27000)
//                ,new WaitCommand(250)
                ,new InstantCommand(()-> m_robot.m_shooter.setTargetVel(0))
        );
    }

    private SequentialCommandGroup intakeSpikeMark(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> m_robot.m_turntable.setPos(60))
                ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, lineUpSpikeMark)
//                ,new WaitCommand(500)
                ,new InstantCommand(()-> m_robot.m_turntable.setPos(180))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, intakeBallOne)
//                ,new WaitCommand(500)
                ,new InstantCommand(()-> m_robot.m_turntable.setPos(300))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, intakeBallTwo)
//                ,new WaitCommand(500)
                ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
        );
    }

    private SequentialCommandGroup secondVolley(){
        return new SequentialCommandGroup(
                new InstantCommand(()->m_robot.m_turntable.setPos(120))
                ,new WaitCommand(75)
                ,new CMD_AutoColorSwap(m_robot.m_turntable)
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, secondVolleyTraj)
                ,new CMD_ShootAll(m_robot.m_shooter, m_robot.m_turntable, 27000)
//                ,new WaitCommand(250)
                ,new InstantCommand(()-> m_robot.m_shooter.setTargetVel(0))
        );
    }

    private SequentialCommandGroup thirdVolley(){
        return new SequentialCommandGroup(
                new InstantCommand(()->m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, secondIntakeAllign)
                ,new InstantCommand(()-> m_robot.m_turntable.setPos(60))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, secondIntakeBallOne)
                ,new InstantCommand(()-> m_robot.m_turntable.setPos(180))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, secondIntakeBallTwo)
                ,new InstantCommand(()-> m_robot.m_turntable.setPos(300))
                ,new InstantCommand(()->m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, thirdVolleyTraj)
                ,new CMD_ShootAll(m_robot.m_shooter,m_robot.m_turntable,m_robot.drivetrain,m_robot.m_vision)
        );
    }
    private SequentialCommandGroup fourthVolley(){
        return new SequentialCommandGroup(
                new RR_TrajectoryFollowerCommand(m_robot.drivetrain,thirdIntakeAllign)
                ,new CMD_IntakeToggle(m_robot.m_intake,m_robot.m_turntable)
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain,thirdIntake)
                ,new CMD_IntakeToggle(m_robot.m_intake,m_robot.m_turntable)
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, fourthVolleyTraj)
                ,new CMD_ShootAll(m_robot.m_shooter, m_robot.m_turntable,m_robot.drivetrain,m_robot.m_vision)
        );
    }
    private SequentialCommandGroup park(){
        return new SequentialCommandGroup(
                new InstantCommand(()->m_robot.m_shooter.setTargetVel(0))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain,park)
        );
    }
}