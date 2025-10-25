package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class RobotContainer {
     public boolean m_red = true;
     public Robot m_robot = new Robot();
     public MecanumDriveSubsystem drivetrain;
     public GlobalVariables GlobalVariables;
     public SUB_Shooter m_shooter;
     public SUB_Intake m_intake;
     public SUB_Vision m_vision;
     public SUB_Lift m_lift;

     public RobotContainer(OpMode p_opMode) {
          drivetrain = new MecanumDriveSubsystem(new SampleMecanumDrive(p_opMode.hardwareMap), true);
          GlobalVariables = new GlobalVariables();
          m_intake = new SUB_Intake(p_opMode);
          m_shooter = new SUB_Shooter(p_opMode);
          m_vision = new SUB_Vision(p_opMode, GlobalVariables);
          m_lift = new SUB_Lift(p_opMode);
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

     public boolean getRedSide(){
          return m_red;
     }
}

