package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class CMD_TargetLock extends CommandBase {

    // =========================== CONSTANTS ===============================

    private static final double kP_TURN = 0.035;      // PID proportional gain for yaw correction
    private static final double RIGHT_STICK_CANCEL = 0.25;

    // Correct FTC DECODE Center Goal Tags
    private static final int BLUE_CENTER_TAG = 20;
    private static final int RED_CENTER_TAG = 24;

    // =====================================================================

    private final MecanumDriveSubsystem m_drive;
    private final SUB_Vision m_vision;
    private final GamepadEx m_driver;

    private boolean m_finished = false;

    public CMD_TargetLock(MecanumDriveSubsystem drive, SUB_Vision vision, GamepadEx driver) {
        m_drive = drive;
        m_vision = vision;
        m_driver = driver;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_finished = false;
        m_drive.stop();
    }

    @Override
    public void execute() {

        // ------------------ DRIVER OVERRIDE CHECK -------------------------
        double rightMag = Math.hypot(m_driver.getRightX(), m_driver.getRightY());

        if (rightMag > RIGHT_STICK_CANCEL) {
            m_finished = true;
            return;
        }

        // ------------------ GET TARGET TAG ID ------------------------------
        int targetId = GlobalVariables.m_red ? RED_CENTER_TAG : BLUE_CENTER_TAG;

        AprilTagDetection detection = null;

        if (m_vision.getDetections() != null) {
            for (AprilTagDetection det : m_vision.getDetections()) {
                if (det.id == targetId) {
                    detection = det;
                    break;
                }
            }
        }

        // ------------------ NO TAG → CANCEL -------------------------
        if (detection == null) {
            m_finished = true;
            return;
        }

        // ------------------ YAW ERROR → TURN POWER ------------------------
        double yawErrorDeg = detection.ftcPose.yaw;
        double turnPower = yawErrorDeg * kP_TURN;

        // ------------------ DRIVER TRANSLATION ----------------------------
        double x = m_driver.getLeftX();
        double y = -m_driver.getLeftY();

        m_drive.setDrivePower(
                new Pose2d(
                        y
                        ,x
                        ,turnPower
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
