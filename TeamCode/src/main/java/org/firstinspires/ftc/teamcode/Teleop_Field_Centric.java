package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Teleop Red", group ="Teleop Red")
public class Teleop_Field_Centric extends LinearOpMode {

     public RobotContainer m_robot;
     private GamepadEx m_driverOp;
     private GamepadEx m_toolOp;

     private static ElapsedTime m_runTime = new ElapsedTime();

     public void initialize() {
          telemetry.clearAll();
          telemetry.addData("init complete", true);

          m_runTime.reset();
     }

     @Override
     public void runOpMode() throws InterruptedException {
          initializeSubsystems();
          // waitForStart();
          while (!opModeIsActive() && !isStopRequested()) {
               telemetry.update();
          }

          m_runTime.reset();
          while (!isStopRequested() && opModeIsActive()) {
               m_robot.run(); // run the scheduler

               m_robot.drivetrain.update();
               Pose2d poseEstimate = m_robot.drivetrain.getPoseEstimate();
               telemetry.addData("ODM","x[%3.2f] y[%3.2f] heading(%3.2f)", poseEstimate.getX(),
                       poseEstimate.getY(), Math.toDegrees(poseEstimate.getHeading()));

               telemetry.addData("RobotState", m_robot.GlobalVariables.getRobotState().name());
               telemetry.update();
          }

          //
          endOfOpMode();
          m_robot.reset();
     }

     public void endOfOpMode() {

     }

     public void initializeSubsystems() {
          m_robot = new RobotContainer(this);
          m_driverOp = new GamepadEx(gamepad1);
          m_toolOp = new GamepadEx(gamepad2);

          setSide();

          //drivetrain initialization
          m_robot.drivetrain.setFieldCentric(true);
          m_robot.drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          m_robot.drivetrain.setDefaultCommand(new RR_MecanumDriveDefault(m_robot.drivetrain, m_driverOp,
                  m_toolOp, 0.0,0.01, m_robot.GlobalVariables));

          m_robot.m_colorSensor.setDefaultCommand(new CMD_LedDefault(m_robot.m_colorSensor));
          //button bindings and global variables initialization
          configureButtonBindings();
     }

     public void configureButtonBindings() {

     }

     public void setSide() {
          m_robot.setRedSide();
     }

     public void AddButtonCommand(GamepadEx gamepad, GamepadKeys.Button button, Command command) {
          (new GamepadButton(gamepad, button)).whenPressed(command);
     }

     public void AddButtonCommandNoInt(GamepadEx gamepad, GamepadKeys.Button button, Command command) {
          (new GamepadButton(gamepad, button)).whenPressed(command, false);
     }
}