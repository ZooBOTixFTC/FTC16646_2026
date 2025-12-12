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

@Autonomous(name = "Close Blue", preselectTeleOp = "Teleop Blue", group = "Auto Blue")
public class AUTO_CloseBlue extends Robot_Auto {
    private Trajectory firstVolley,  lineupFirstSpikeMark, intakeFirstSpikeMark, secondVolley,
            lineupSecondSpikeMark, intakeSecondSpikeMark, thirdVolley, lineupThirdSpikeMark,
            intakeThirdSpikeMark, fourthVolley, park;

    @Override
    public void prebuildTasks() {
        setStartingPose(new Pose2d(50, 47, Math.toRadians(50)));
        GlobalVariables.m_far = false;
        GlobalVariables.m_red = false;

        firstVolley = m_robot.drivetrain.trajectoryBuilder(getStartingPose(), true)
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(43)))
                .build();

        lineupFirstSpikeMark = m_robot.drivetrain.trajectoryBuilder(firstVolley.end(), false)
                .lineToLinearHeading(new Pose2d(14, 28,  Math.toRadians(90)))
                .build();

        intakeFirstSpikeMark = m_robot.drivetrain.trajectoryBuilder(lineupFirstSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(3, 53, Math.toRadians(90)))
                .build();

        secondVolley = m_robot.drivetrain.trajectoryBuilder(intakeFirstSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(43)))
                .build();

        lineupSecondSpikeMark = m_robot.drivetrain.trajectoryBuilder(secondVolley.end(), false)
                .lineToLinearHeading(new Pose2d(-10, 30,  Math.toRadians(90)))
                .build();

        intakeSecondSpikeMark = m_robot.drivetrain.trajectoryBuilder(lineupSecondSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(-16, 59, Math.toRadians(90)))
                .build();

        thirdVolley = m_robot.drivetrain.trajectoryBuilder(intakeSecondSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(43)))
                .build();

        lineupThirdSpikeMark = m_robot.drivetrain.trajectoryBuilder(thirdVolley.end(), false)
                .lineToLinearHeading(new Pose2d(-32, 30, Math.toRadians(90)))
                .build();

        intakeThirdSpikeMark = m_robot.drivetrain.trajectoryBuilder(lineupThirdSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(-38, 60, Math.toRadians(90)))
                .build();

        fourthVolley = m_robot.drivetrain.trajectoryBuilder(intakeThirdSpikeMark.end(), false)
                .lineToLinearHeading(new Pose2d(10, 18, Math.toRadians(42)))
                .build();

        park = m_robot.drivetrain.trajectoryBuilder(fourthVolley.end(), false)
                .lineToLinearHeading(new Pose2d(-12, 16, Math.toRadians(90)))
                .build();
    }

    @Override
    public SequentialCommandGroup buildTasks() {
        return new SequentialCommandGroup(
                new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kMidFieldVel))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, firstVolley)
                ,volley()
                ,intakeFirstSpikeMark()
                ,volley()
                ,intakeSecondSpikeMark()
                ,volley()
                ,intakeThirdSpikeMark()
                ,volley()
                //park
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, park)
        );
    }

    private SequentialCommandGroup volley(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
                ,new CMD_ShootAuto(m_robot.m_shooter, m_robot.m_kicker, m_robot.m_intake)
        );
    }

    private SequentialCommandGroup intakeFirstSpikeMark(){
        return new SequentialCommandGroup(
                new InstantCommand(m_robot.m_shooter::setStopperClosed)
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, lineupFirstSpikeMark)
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, intakeFirstSpikeMark)
                ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kMidFieldVel))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, secondVolley)
        );
    }

    private SequentialCommandGroup intakeSecondSpikeMark(){
        return new SequentialCommandGroup(
                new InstantCommand(m_robot.m_shooter::setStopperClosed)
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, lineupSecondSpikeMark)
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, intakeSecondSpikeMark)
                ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kMidFieldVel))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, thirdVolley)
        );
    }

    private SequentialCommandGroup intakeThirdSpikeMark(){
        return new SequentialCommandGroup(
                new InstantCommand(m_robot.m_shooter::setStopperClosed)
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, lineupThirdSpikeMark)
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, intakeThirdSpikeMark)
                ,new InstantCommand(()-> m_robot.m_shooter.setShootingVelocity(Constants.ShooterConstants.kMidFieldVel))
                ,new RR_TrajectoryFollowerCommand(m_robot.drivetrain, fourthVolley)
        );
    }
}