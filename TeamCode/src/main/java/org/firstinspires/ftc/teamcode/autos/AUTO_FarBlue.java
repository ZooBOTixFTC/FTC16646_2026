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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Autonomous(name = "Far Blue", preselectTeleOp = "Teleop Blue", group = "Auto Blue")
public class AUTO_FarBlue extends Robot_Auto {

    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(-64.75, 9.25, Math.toRadians(0)));
        GlobalVariables.m_far = true;
        GlobalVariables.m_red = false;
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
                new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kPreRev))
                ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-60, 16, Math.toRadians(20)), false, DriveConstants.MAX_VEL)
                ,new CMD_AlignTarget(m_robot.drivetrain, m_robot.m_vision)
                ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kFarVel))
                ,new CMD_Shoot(m_robot.m_shooter, m_robot.m_kicker, m_robot.m_intake)
                ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(0))
        );
    }

    private SequentialCommandGroup intakeFirstSpikeMark(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
                ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-37, 30, Math.toRadians(90)), false, DriveConstants.MAX_VEL)
                ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-32, 54, Math.toRadians(90)), false, DriveConstants.MAX_VEL/3)
                ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(-.1))
                ,new WaitCommand(67)
                ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
        );
    }

    private SequentialCommandGroup secondVolley(){
        return new SequentialCommandGroup(
                new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-55, 12, Math.toRadians(20)), false, DriveConstants.MAX_VEL)
                ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kPreRev))
                ,new CMD_AlignTarget(m_robot.drivetrain, m_robot.m_vision)
                ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kFarVel))
                ,new CMD_Shoot(m_robot.m_shooter, m_robot.m_kicker, m_robot.m_intake)
                ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(0))
        );
    }

    private SequentialCommandGroup intakeSecondSpikeMark(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
                ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-13, 30, Math.toRadians(90)), false, DriveConstants.MAX_VEL)
                ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-6, 54, Math.toRadians(90)), false, DriveConstants.MAX_VEL/3)
                ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(-.1))
                ,new WaitCommand(67)
                ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
        );
    }

    private SequentialCommandGroup thirdVolley(){
        return new SequentialCommandGroup(
                new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-50, 12, Math.toRadians(22.5)), false, DriveConstants.MAX_VEL)
                ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kPreRev))
                ,new CMD_AlignTarget(m_robot.drivetrain, m_robot.m_vision)
                ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kFarVel))
                ,new CMD_Shoot(m_robot.m_shooter, m_robot.m_kicker, m_robot.m_intake)
                ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(0))
                ,new RR_TrajectoryLineToLinearHeading(m_robot.drivetrain, new Pose2d(-40, 12, Math.toRadians(0)), false, DriveConstants.MAX_VEL)
        );
    }
}
