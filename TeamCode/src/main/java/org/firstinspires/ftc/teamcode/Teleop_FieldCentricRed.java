package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.*;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Teleop Red", group ="Teleop Red")
public class Teleop_FieldCentricRed extends LinearOpMode {

    public RobotContainer m_robot;
    private GamepadEx m_driverOp;
    private GamepadEx m_toolOp;

    private final  ElapsedTime m_runTime = new ElapsedTime();

    public void initialize() {
        telemetry.clearAll();
        telemetry.addData("init complete", true);

        m_runTime.reset();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeSubsystems();

        while (!opModeIsActive() && !isStopRequested()) {
            m_robot.m_limelight.updateDuringInit();//get vision updates during init phase
            telemetry.update();//update telemetry during init
        }

        m_runTime.reset();
        while (!isStopRequested() && opModeIsActive()) {
            m_robot.run(); // run the scheduler

            Pose2d poseEstimate = m_robot.m_drivetrain.getDrivePose();
            telemetry.addData("ODM","x[%3.2f] y[%3.2f] heading(%3.2f)", poseEstimate.position.x,
                    poseEstimate.position.y, poseEstimate.heading);

            telemetry.update();
        }

        endOfOpMode();
        m_robot.reset();
    }

    public void endOfOpMode() {

    }

    public void initializeSubsystems() {
        m_robot = new RobotContainer(this, GlobalVariables.m_filteredPose);
        m_driverOp = new GamepadEx(gamepad1);
        m_toolOp = new GamepadEx(gamepad2);

        setSide();

        //drivetrain init
        m_robot.m_drivetrain.setDefaultCommand(new RR_MecanumDriveDefault(m_robot.m_drivetrain, m_driverOp
                ,GlobalVariables.m_redSide ? -90 : 90,0.01));
        //button bindings and global variables initialization
        configureButtonBindings();
    }

    public void configureButtonBindings() {

    }

    public void setSide() {
        GlobalVariables.m_redSide = true;
    }

    public void AddButtonCommand(GamepadEx gamepad, GamepadKeys.Button button, Command command) {
        (new GamepadButton(gamepad, button)).whenPressed(command);
    }

    public void AddButtonCommandNoInt(GamepadEx gamepad, GamepadKeys.Button button, Command command) {
        (new GamepadButton(gamepad, button)).whenPressed(command, false);
    }
}