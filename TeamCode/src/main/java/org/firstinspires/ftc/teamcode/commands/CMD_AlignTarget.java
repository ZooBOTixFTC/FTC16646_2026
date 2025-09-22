package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class CMD_AlignTarget extends CommandBase {
    private final MecanumDriveSubsystem m_drive;
    private final SUB_Vision m_vision;
    private final GlobalVariables m_variables;

    private double m_targetHeadingDeg = 0.0;
    private boolean m_snapshotTaken = false;
    private boolean isFinished;

    public CMD_AlignTarget(MecanumDriveSubsystem drive, SUB_Vision vision, GlobalVariables variables) {
        this.m_drive = drive;
        this.m_vision = vision;
        this.m_variables = variables;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        isFinished = false;
        int targetId = m_variables.m_red ? 24 : 20;

        for (AprilTagDetection detection : m_vision.getDetections()) {
            if (detection.id == targetId) {
                double currentHeadingDeg = Math.toDegrees(m_drive.getPoseEstimate().getHeading());

                double tagX = detection.ftcPose.x;
                double tagY = detection.ftcPose.y;
                double angleToTagDeg = Math.toDegrees(Math.atan2(tagY, tagX));

                angleToTagDeg = (angleToTagDeg + 115) % 360;

                m_targetHeadingDeg = (currentHeadingDeg + angleToTagDeg) % 360;
                m_snapshotTaken = true;
                break;
            }
        }
    }

    @Override
    public void execute() {
        if (!m_snapshotTaken) return;

        double currentHeadingDeg = Math.toDegrees(m_drive.getPoseEstimate().getHeading());
        double error = m_targetHeadingDeg - currentHeadingDeg;

        // Normalize error to [-180, 180]
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        double kP = 0.015;
        double turnPower = 0.0;
        if(error > 5) {
            turnPower = kP * error;
        }else{
            isFinished = true;
        }

        m_drive.drive(0.0, 0.0, turnPower);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return !m_snapshotTaken || isFinished;
    }
}
