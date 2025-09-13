package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ftclib.command.Command;
import org.firstinspires.ftc.teamcode.ftclib.command.Robot;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.visionprocessor.VProcessorDetectBlock;

public class RobotContainer {
     public boolean m_red = true;
     public Robot m_robot = new Robot();
     public MecanumDriveSubsystem drivetrain;
     public ColorSubsystem colorsensor;
     public VisionSubsystem vision;
     public GlobalVariables GlobalVariables;

     public RobotContainer(OpMode p_opMode) {
          drivetrain = new MecanumDriveSubsystem(new SampleMecanumDrive(p_opMode.hardwareMap), true);
          GlobalVariables = new GlobalVariables();
          colorsensor = new ColorSubsystem(p_opMode);
          vision = new VisionSubsystem(p_opMode.hardwareMap);
     }

     public void run() {
          m_robot.run();
     }

     public void reset() {
          m_robot.reset();
     }

     public void schedule(Command... commands) {
          m_robot.schedule(commands);
     }

     public void setRedSide() {
          m_red = true;
     }

     public void setBlueSide() {
          m_red = false;
     }
}

