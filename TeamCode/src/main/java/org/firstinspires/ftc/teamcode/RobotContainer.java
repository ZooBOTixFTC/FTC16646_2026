package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ftclib.command.Command;
import org.firstinspires.ftc.teamcode.ftclib.command.Robot;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.visionprocessor.VProcessorDetectBlock;

public class RobotContainer {
     public boolean m_red = true;
     public Robot m_robot = new Robot();
     public MecanumDriveSubsystem drivetrain;
     public GlobalVariables GlobalVariables;
     public SUB_Shooter shooter;
     public SUB_Intake intake;
     public SUB_Lift lift;

     public RobotContainer(OpMode p_opMode) {
          drivetrain = new MecanumDriveSubsystem(new SampleMecanumDrive(p_opMode.hardwareMap), true);
          GlobalVariables = new GlobalVariables();
          intake = new SUB_Intake(p_opMode);
          shooter = new SUB_Shooter(p_opMode);
          lift = new SUB_Lift(p_opMode);
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

     public void setRedSide() {
          m_red = true;
     }

     public void setBlueSide() {
          m_red = false;
     }
}

