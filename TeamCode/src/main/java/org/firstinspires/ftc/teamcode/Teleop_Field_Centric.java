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

          m_robot.drivetrain.setPoseEstimate(GlobalVariables.m_autoEndPose);
          m_robot.m_vision.stream(true);

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
                  m_robot.getRedSide() ? -90 : 90,0.05, m_robot.GlobalVariables));

          if (!Constants.ShooterConstants.kTuningMode) m_robot.m_shooter.setDefaultCommand(new CMD_ShooterDefault(m_robot.m_shooter, m_robot.m_intake));

          configureButtonBindings();
     }

     public void configureButtonBindings() {
          AddTriggerCommand(m_driverOp, GamepadKeys.Trigger.RIGHT_TRIGGER, new CMD_Shoot(m_robot.drivetrain,
                  m_robot.m_shooter, m_robot.m_lift, m_robot.m_intake, m_robot.m_vision, m_driverOp));
          AddTriggerToggleCommand(m_driverOp, GamepadKeys.Trigger.LEFT_TRIGGER,
                  new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
                  ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
          );
//          AddButtonCommand(m_driverOp, GamepadKeys.Button.LEFT_BUMPER, new CMD_ShootLeft(m_robot.m_shooter, m_robot.m_lift));
//          AddButtonCommand(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER, new CMD_ShootRight(m_robot.m_shooter, m_robot.m_lift));

          AddButtonCommand(m_driverOp, GamepadKeys.Button.Y, new CMD_AlignTarget(
                  m_robot.drivetrain, m_robot.m_vision, m_driverOp));

          AddButtonToggleCommand(m_driverOp, GamepadKeys.Button.BACK,
                  new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeReverse))
                  ,new InstantCommand(()-> m_robot.m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff)));

          AddButtonCommand(m_driverOp, GamepadKeys.Button.START,
               new InstantCommand(()-> m_robot.m_shooter.setTargetVelLeft(0))
               .alongWith(new InstantCommand(()-> m_robot.m_shooter.setTargetVelRight(0)))
          );
     }

     public void setSide() {
          m_robot.setRedSide();
     }

     public void AddButtonCommand(GamepadEx gamepad, GamepadKeys.Button button, Command command) {
          new GamepadButton(gamepad, button).whenPressed(command);
     }

     public void AddButtonToggleCommand(GamepadEx gamepad, GamepadKeys.Button button, Command onTrue, Command onFalse){
          new GamepadButton(gamepad, button).whenPressed(onTrue).whenReleased(onFalse);
     }

     public void AddTriggerCommand(GamepadEx gamepad, GamepadKeys.Trigger trigger, Command command){
          new Trigger(()-> gamepad.getTrigger(trigger) >= .5).whenActive(command);
     }

     public void AddTriggerToggleCommand(GamepadEx gamepad, GamepadKeys.Trigger trigger, Command onTrue, Command onFalse){
          new Trigger(()-> gamepad.getTrigger(trigger) >= .5).whenActive(onTrue).whenInactive(onFalse);
     }
}