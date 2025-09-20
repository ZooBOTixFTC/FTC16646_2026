package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class CMD_AlignTarget extends CommandBase {
    private final MecanumDriveSubsystem m_drive;
    private final SUB_Vision m_vision;
    private final GlobalVariables m_variables;
    private final GamepadEx m_driverOp;

    private double m_targetYawDeg = 0.0;
    private boolean m_snapshotTaken = false;

    public CMD_AlignTarget(MecanumDriveSubsystem p_drive, SUB_Vision p_vision,
                           GlobalVariables p_variables, GamepadEx p_driverOp) {
        this.m_drive = p_drive;
        this.m_vision = p_vision;
        this.m_variables = p_variables;
        this.m_driverOp = p_driverOp;

        addRequirements(p_drive);
    }

    @Override
    public void initialize() {
        int targetId = m_variables.m_red ? 24 : 20;

        for (AprilTagDetection detection : m_vision.getDetections()) {
            if (detection.id == targetId) {
                m_targetYawDeg = detection.ftcPose.yaw;
                m_snapshotTaken = true;
                break;
            }
        }

        if (m_snapshotTaken) {
            // Convert relative yaw to absolute heading
            m_targetYawDeg += Math.toDegrees(m_drive.getPoseEstimate().getHeading());
        }
    }

    @Override
    public void execute() {
        if (!m_snapshotTaken) return;

        double currentHeadingDeg = Math.toDegrees(m_drive.getPoseEstimate().getHeading());
        double error = m_targetYawDeg - currentHeadingDeg;

        // Normalize error to [-180, 180]
        error = ((error + 180) % 360 + 360) % 360 - 180;

        double kP = 0.01;
        double turnPower = kP * error;

        // Clamp turn power
        turnPower = Math.max(-0.5, Math.min(0.5, turnPower));

        // Drive with driver input + alignment correction
        m_drive.drive(m_driverOp.getLeftY(), m_driverOp.getLeftX(), turnPower);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        if (!m_snapshotTaken) return true;

        double currentHeadingDeg = Math.toDegrees(m_drive.getPoseEstimate().getHeading());
        double error = Math.abs(m_targetYawDeg - currentHeadingDeg);
        return !m_driverOp.getButton(GamepadKeys.Button.B) || error < 2.0;
    }
}
