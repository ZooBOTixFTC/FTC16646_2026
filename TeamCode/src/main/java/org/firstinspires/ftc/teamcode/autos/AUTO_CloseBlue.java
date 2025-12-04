package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Close Blue", preselectTeleOp = "Teleop Blue", group = "Auto Blue")
public class AUTO_CloseBlue extends Robot_Auto {

    private Trajectory firstVolley, lineupFirstSpikeMark, intakeFirstSpikeMark, secondVolley,
            lineupSecondSpikeMark, intakeSecondSpikeMark, thirdVolley, park;

    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(41.5, 54.5, Math.toRadians(0)));
        GlobalVariables.m_far = false;
        GlobalVariables.m_red = false;

        firstVolley = m_robot.drivetrain.trajectoryBuilder(getStartingPose(), false)
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(45)))
                .build();

        lineupFirstSpikeMark = m_robot.drivetrain.trajectoryBuilder(firstVolley.end(), false)
                .lineToLinearHeading(new Pose2d(12, 30, Math.toRadians(90)))
                .build();

        intakeFirstSpikeMark = m_robot.drivetrain.trajectoryBuilder(lineupFirstSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(8, 50, Math.toRadians(90))
                ,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH)
                ,SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .build();

        secondVolley = m_robot.drivetrain.trajectoryBuilder(intakeFirstSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(45)))
                .build();

        lineupSecondSpikeMark = m_robot.drivetrain.trajectoryBuilder(secondVolley.end(), false)
                .lineToLinearHeading(new Pose2d(-13, 30, Math.toRadians(90)))
                .build();

        intakeSecondSpikeMark = m_robot.drivetrain.trajectoryBuilder(lineupSecondSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(-18, 54, Math.toRadians(90))
                ,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH)
                ,SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .build();

        thirdVolley = m_robot.drivetrain.trajectoryBuilder(intakeSecondSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(45)))
                .build();

        park = m_robot.drivetrain.trajectoryBuilder(thirdVolley.end(), false)
                .strafeLeft(24)
                .build();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        return new SequentialCommandGroup(
                firstVolley()
                ,intakeFirstSpikeMark()
                ,secondVolley()
                ,intakeSecondSpikeMark()
                ,thirdVolley()
        );
    }

    private SequentialCommandGroup firstVolley(){
        return new SequentialCommandGroup(
            new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kMidFieldVel))
            ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, firstVolley)
            ,new CMD_Shoot(m_robot.m_shooter, m_robot.m_kicker, m_robot.m_intake)
            ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(0))
        );
    }

    private SequentialCommandGroup intakeFirstSpikeMark(){
        return new SequentialCommandGroup(
            new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
            ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, lineupFirstSpikeMark)
            ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, intakeFirstSpikeMark)
            ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(-.1))
            ,new WaitCommand(67)
            ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
        );
    }

    private SequentialCommandGroup secondVolley(){
        return new SequentialCommandGroup(
            new RR_TrajectoryFollowerCommand(m_robot.drivetrain, secondVolley)
            ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kMidFieldVel))
            ,new CMD_Shoot(m_robot.m_shooter, m_robot.m_kicker, m_robot.m_intake)
            ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(0))
        );
    }

    private SequentialCommandGroup intakeSecondSpikeMark(){
        return new SequentialCommandGroup(
            new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
            ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, lineupSecondSpikeMark)
            ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, intakeSecondSpikeMark)
            ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(-.1))
            ,new WaitCommand(67)
            ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
        );
    }

    private SequentialCommandGroup thirdVolley(){
        return new SequentialCommandGroup(
            new RR_TrajectoryFollowerCommand(m_robot.drivetrain, thirdVolley)
            ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kMidFieldVel))
            ,new CMD_Shoot(m_robot.m_shooter, m_robot.m_kicker, m_robot.m_intake)
            ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(0))
            ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, park)
        );
    }
}
