package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Robot_Auto;
import org.firstinspires.ftc.teamcode.commands.*;

@Autonomous(name = "Far Red", preselectTeleOp = "Teleop Red", group = "Auto Red")
public class AUTO_FarRed extends Robot_Auto {

    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-64.75, -9.25, Math.toRadians(0)));
        GlobalVariables.m_red = true;
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
            new InstantCommand(()-> m_robot.m_shooter.setGoal(2000))
            ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-60, -16, Math.toRadians(-20)), false)
            ,new CMD_AlignTarget(m_robot.drivetrain, m_robot.m_vision)
            ,new InstantCommand(()-> m_robot.m_shooter.setGoal(Constants.ShooterConstants.kMaxVel))
            ,new WaitCommand(500)
            ,new CMD_GetShooterAtVelocity(m_robot.m_shooter)
            ,new CMD_Kick(m_robot.m_lift)
            ,new WaitCommand(500)
            ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
            ,new CMD_Kick(m_robot.m_lift)
            ,new WaitCommand(750)
            ,new InstantCommand(()-> m_robot.m_shooter.setGoal(0))
        );
    }

    private SequentialCommandGroup intakeFirstSpikeMark(){
        return new SequentialCommandGroup(
            new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeAuto))
            ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-36, -30, Math.toRadians(-90)), false)
            ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-32, -54, Math.toRadians(-90)), false)
            ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(-.1))
            ,new WaitCommand(67)
            ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
        );
    }

    private SequentialCommandGroup secondVolley(){
        return new SequentialCommandGroup(
            new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-60, -16, Math.toRadians(-20)), false)
            ,new InstantCommand(()-> m_robot.m_shooter.setGoal(2000))
            ,new CMD_AlignTarget(m_robot.drivetrain, m_robot.m_vision)
            ,new InstantCommand(()-> m_robot.m_shooter.setGoal(Constants.ShooterConstants.kMaxVel))
            ,new CMD_GetShooterAtVelocity(m_robot.m_shooter)
            ,new CMD_Kick(m_robot.m_lift)
            ,new WaitCommand(500)
            ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
            ,new CMD_Kick(m_robot.m_lift)
            ,new WaitCommand(750)
            ,new InstantCommand(()-> m_robot.m_shooter.setGoal(0))
        );
    }

    private SequentialCommandGroup intakeSecondSpikeMark(){
        return new SequentialCommandGroup(
            new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeAuto))
            ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-12, -30, Math.toRadians(-90)), false)
            ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-8, -54, Math.toRadians(-90)), false)
            ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(-.1))
            ,new WaitCommand(67)
            ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
        );
    }

    private SequentialCommandGroup thirdVolley(){
        return new SequentialCommandGroup(
                new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-60, -16, Math.toRadians(-20)), false)
                ,new InstantCommand(()-> m_robot.m_shooter.setGoal(2000))
                ,new CMD_AlignTarget(m_robot.drivetrain, m_robot.m_vision)
                ,new InstantCommand(()-> m_robot.m_shooter.setGoal(Constants.ShooterConstants.kMaxVel))
                ,new CMD_GetShooterAtVelocity(m_robot.m_shooter)
                ,new CMD_Kick(m_robot.m_lift)
                ,new WaitCommand(500)
                ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
                ,new CMD_Kick(m_robot.m_lift)
                ,new WaitCommand(750)
                ,new InstantCommand(()-> m_robot.m_shooter.setGoal(0))
        );
    }
}
