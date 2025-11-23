package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
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

     private static final ElapsedTime m_runTime = new ElapsedTime();

     public void initialize() {
          telemetry.clearAll();
          telemetry.addData("init complete", true);

          m_runTime.reset();
     }

     @Override
     public void runOpMode() throws InterruptedException {
          initializeSubsystems();

          m_robot.drivetrain.setPoseEstimate(new Pose2d(
                  GlobalVariables.m_autoEndPose.getX()
                  ,GlobalVariables.m_autoEndPose.getY()
                  ,GlobalVariables.m_red ? Math.toRadians(-90) : Math.toRadians(90)
          ));

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

               telemetry.update();
          }

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
          GlobalVariables.m_red = m_robot.getRedSide();

          m_robot.drivetrain.setFieldCentric(true);
          m_robot.drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          m_robot.drivetrain.setDefaultCommand(new RR_MecanumDriveDefault(m_robot.drivetrain, m_driverOp,
                  GlobalVariables.m_red ? -90 : 90,0.05, m_robot.GlobalVariables));

          m_robot.m_shooter.setDefaultCommand(new CMD_ShooterDefault(m_robot.m_shooter));

          configureButtonBindings();
     }

     public void configureButtonBindings() {
          AddButtonCommand(m_driverOp, GamepadKeys.Button.A, new CMD_Shoot(m_robot.m_shooter, m_robot.m_turntable));

          AddTriggerCommand(m_driverOp, GamepadKeys.Trigger.RIGHT_TRIGGER, new CMD_ShootAll(m_robot.m_shooter,m_robot.m_turntable));

          AddButtonCommand(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER, new InstantCommand(()-> m_robot.m_turntable.rotateRight()));
          AddButtonCommand(m_driverOp, GamepadKeys.Button.LEFT_BUMPER, new InstantCommand(()-> m_robot.m_turntable.rotateLeft()));
          AddTriggerCommand(m_driverOp, GamepadKeys.Trigger.LEFT_TRIGGER, new CMD_IntakeToggle(m_robot.m_intake, m_robot.m_turntable));
          AddButtonCommand(m_driverOp, GamepadKeys.Button.Y, new CMD_AlignTarget(m_robot.drivetrain, m_robot.m_vision, m_driverOp));

          AddButtonCommand(m_driverOp, GamepadKeys.Button.START, new InstantCommand(()-> m_robot.m_shooter.setTargetVel(0)));
          AddButtonCommand(m_driverOp, GamepadKeys.Button.BACK, new CMD_IntakeReverse(m_robot.m_intake));

          //Operator

         AddButtonCommand(m_driverOp, GamepadKeys.Button.B,new CMD_Kick(m_robot.m_shooter,m_robot.m_turntable));
         AddButtonCommand(m_toolOp, GamepadKeys.Button.START, new InstantCommand(()->
                  m_robot.drivetrain.setPoseEstimate(new Pose2d(
                     m_robot.drivetrain.getPoseEstimate().getX(),
                     m_robot.drivetrain.getPoseEstimate().getY(),
                     0))));

         AddButtonCommandToggle(m_toolOp, GamepadKeys.Button.BACK,
             new InstantCommand(()-> m_robot.m_turntable.setResetMode()),
             new InstantCommand(()-> m_robot.m_turntable.setReset()));

         AddButtonCommand(m_toolOp, GamepadKeys.Button.A, new CMD_AutoColorSwap(m_robot.m_turntable));
         AddButtonCommand(m_toolOp, GamepadKeys.Button.X, new InstantCommand(()-> m_robot.m_shooter.setTargetVel(25000)));
     }

     public void setSide() {
          m_robot.setRedSide();
     }

     public void AddButtonCommand(GamepadEx gamepad, GamepadKeys.Button button, Command command) {
          new GamepadButton(gamepad, button).whenPressed(command);
     }

     public void AddButtonCommandToggle(GamepadEx gamepad, GamepadKeys.Button button, Command onTrue, Command onFalse) {
          new GamepadButton(gamepad, button).whenPressed(onTrue).whenReleased(onFalse);
     }

     public void AddTriggerCommand(GamepadEx gamepad, GamepadKeys.Trigger trigger, Command command){
          new Trigger(()-> gamepad.getTrigger(trigger) >= .5).whenActive(command);
     }
}