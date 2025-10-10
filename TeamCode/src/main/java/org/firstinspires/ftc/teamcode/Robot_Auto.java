package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Robot_Auto extends LinearOpMode {

     public RobotContainer m_robot;

     private Pose2d m_startingPose = new Pose2d(0, 0, 0);

     private final ElapsedTime m_runTime = new ElapsedTime();

     public void initialize() {
          telemetry.clearAll();
          telemetry.addData("init complete", "BaseRobot");
     }

     @Override
     public void runOpMode() throws InterruptedException {
          initializeSubsystems();

//          m_robot.m_vision.stream(true);
//          m_robot.m_vision.readPattern();

          prebuildTasks();

          while (!opModeIsActive() && !isStopRequested()) {
               m_robot.run(); // run the scheduler
               telemetry.update();
          }

//          m_robot.m_vision.stream(false);

          buildTasks();

          m_runTime.reset();

          while (!isStopRequested() && opModeIsActive()) {
               m_robot.run(); // run the scheduler
               m_robot.drivetrain.update();
               Pose2d poseEstimate = m_robot.drivetrain.getPoseEstimate();
               telemetry.addData("ODM","x[%3.2f] y[%3.2f] heading(%3.2f)", poseEstimate.getX(), poseEstimate.getY(), Math.toDegrees(poseEstimate.getHeading()));
               telemetry.update();
          }

          endOfOpMode();
          m_robot.reset();
     }

     public void endOfOpMode() {
          GlobalVariables.m_autoEndPose = m_robot.drivetrain.getPoseEstimate();
          m_robot.reset();
     }

     public void initializeSubsystems() {
          m_robot = new RobotContainer(this);
     }

     public void setStartingPose(Pose2d p_pose) {
          m_startingPose = p_pose;
          m_robot.drivetrain.setPoseEstimate(m_startingPose);
     }

     public Pose2d getStartingPose() {
          return m_startingPose;
     }

     public abstract SequentialCommandGroup buildTasks();
     public abstract void prebuildTasks();

}