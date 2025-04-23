package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

public class RobotContainer {
    public Robot m_robot = new Robot();
    public final SUB_Drivetrain m_drivetrain;
    public final SUB_Limelight m_limelight;

    public RobotContainer(OpMode p_opMode, Pose2d m_startPose) {
        m_drivetrain = new SUB_Drivetrain(p_opMode, new MecanumDrive(p_opMode.hardwareMap, m_startPose), true);
        m_limelight = new SUB_Limelight(p_opMode, m_drivetrain);
    };

    public void run() {
        m_robot.run();
    }

    public void reset() {
        m_robot.reset();
    }

    public void schedule(Command... commands) {
        m_robot.schedule(commands);
    }
}