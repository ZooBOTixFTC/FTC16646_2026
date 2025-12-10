package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.AutoAlignConstants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Limelight;


public class CMD_AlignTarget extends CommandBase {
    private final MecanumDriveSubsystem m_drive;
    private final SUB_Limelight m_vision;

    private boolean isFinished = false;
    private boolean turnStarted = false;

    public CMD_AlignTarget(MecanumDriveSubsystem drive, SUB_Limelight vision) {
        m_drive = drive;
        m_vision = vision;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        isFinished = false;
        turnStarted = false;
        m_drive.stop();

        // Get target AprilTag detection
        int targetId = GlobalVariables.m_red ? 24 : 20;
        LLResultTypes.FiducialResult currentDetection = null;

        // Find the target AprilTag in current detections
        for (LLResultTypes.FiducialResult detection : m_vision.getLatestResult().getFiducialResults()) {
            if (detection.getFiducialId() == targetId) {
                currentDetection = detection;
                break;
            }
        }

        // If no target tag detected, fail immediately
        if (currentDetection == null) {
            isFinished = true;
            return;
        }

        Pose2d robotPose = m_vision.getPose(AngleUnit.RADIANS);

        Vector2d targetGoal = GlobalVariables.m_red ? AutoAlignConstants.kRedGoalPose : AutoAlignConstants.kBlueGoalPose;

        double desiredHeading = Math.atan2(
                targetGoal.getY() - robotPose.getY(),
                targetGoal.getX() - robotPose.getX()
        );

        turnStarted = true;
        m_drive.turn(Angle.normDelta(desiredHeading - robotPose.getHeading()));
    }

    @Override
    public void execute() {
        // Update RoadRunner drive system (required for async operations)
        m_drive.update();

        // Check if turn is complete
        if (turnStarted && !m_drive.isBusy()) {
            isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Only stop if interrupted AND we actually started a turn
        // RoadRunner automatically stops when turn completes normally
        if (interrupted && turnStarted) {
            m_drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}