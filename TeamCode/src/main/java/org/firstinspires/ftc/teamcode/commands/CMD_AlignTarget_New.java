package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Constants.AutoAlignConstants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


public class CMD_AlignTarget extends CommandBase {
    // ========== TUNABLE PARAMETERS - ADJUST THESE FOR YOUR GOAL ==========

    // Goal geometry (in inches) - MEASURE AND ADJUST THESE VALUES
    private static final double GOAL_WIDTH = 27.0;           // Width of goal opening (inches)
    private static final double GOAL_DEPTH = 18.0;          // Distance from AprilTag to back corner (inches)
    private static final double SHOOTER_OFFSET_X = 0.0;     // Shooter position relative to robot center (inches)
    private static final double SHOOTER_OFFSET_Y = 0.0;     // Shooter distance from robot center forward (inches)

    // Optimization parameters - TUNE THESE FOR BEST PERFORMANCE
    private static final double MIN_SCORING_ANGLE = 5.0;   // Minimum acceptable scoring angle (degrees)
    private static final double SAFETY_MARGIN = 2.0;       // Safety margin from goal edges (inches)
    private static final boolean USE_OPTIMAL_AIMING = true; // Enable/disable optimal aiming

    // Manual override parameters - TUNE THESE FOR DRIVER COMFORT
    private static final double STICK_THRESHOLD = 0.3;     // Minimum stick movement to trigger override

    // =====================================================================

    private final MecanumDriveSubsystem m_drive;
    private final SUB_Vision m_vision;
    private final GamepadEx m_gamepad;

    private boolean isFinished = false;
    private boolean turnStarted = false;

    public CMD_AlignTarget(MecanumDriveSubsystem drive, SUB_Vision vision, GamepadEx gamepad) {
        this.m_drive = drive;
        this.m_vision = vision;
        this.m_gamepad = gamepad;

        addRequirements(drive);
    }

    public CMD_AlignTarget(MecanumDriveSubsystem drive, SUB_Vision vision) {
        this.m_drive = drive;
        this.m_vision = vision;
        this.m_gamepad = null;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        isFinished = false;
        turnStarted = false;

        // Check for manual override first
        if (isManualOverride() || GlobalVariables.m_distToTag < AutoAlignConstants.kDistanceThreshold) {
            isFinished = true;
            return;
        }

        // Get target AprilTag detection
        int targetId = GlobalVariables.m_red ? 24 : 20;
        AprilTagDetection currentDetection = null;

        // Find the target AprilTag in current detections
        for (AprilTagDetection detection : m_vision.getDetections()) {
            if (detection.id == targetId) {
                currentDetection = detection;
                break;
            }
        }

        // If no target tag detected, fail immediately
        if (currentDetection == null) {
            isFinished = true;
            return;
        }

        // Calculate optimal aiming angle (may be different from AprilTag bearing)
        double bearingDegrees = calculateOptimalAiming(currentDetection)
                + (GlobalVariables.m_red ? -1 : 1);

        // Convert to radians and execute turn
        double turnAngleRadians = Math.toRadians(bearingDegrees);
        m_drive.turn(turnAngleRadians);
        turnStarted = true;
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

    /**
     * Calculate optimal aiming angle based on robot position relative to goal
     * @param detection The AprilTag detection
     * @return Optimal bearing angle in degrees (negative = turn left, positive = turn right)
     */
    private double calculateOptimalAiming(AprilTagDetection detection) {
        if (!USE_OPTIMAL_AIMING) {
            return detection.ftcPose.bearing; // Use direct AprilTag aiming if disabled
        }

        // Robot position relative to AprilTag (AprilTag coordinate system)
        double robotX = detection.ftcPose.x;     // Lateral position (+ = right of tag)
        double robotY = detection.ftcPose.range; // Distance to tag (always positive)

        // Account for shooter offset on robot
        double shooterX = robotX - SHOOTER_OFFSET_X;
        double shooterY = robotY - SHOOTER_OFFSET_Y;

        // Goal corners in AprilTag coordinate system
        // AprilTag is at (0, 0), goal extends back and to both sides
        double goalLeftX = -GOAL_WIDTH / 2.0;
        double goalRightX = GOAL_WIDTH / 2.0;
        double goalBackY = -GOAL_DEPTH; // Behind the AprilTag

        // Apply safety margins
        goalLeftX += SAFETY_MARGIN;
        goalRightX -= SAFETY_MARGIN;

        // Calculate angles to goal corners from shooter position
        double angleToLeft = Math.toDegrees(Math.atan2(goalLeftX - shooterX, goalBackY - shooterY));
        double angleToRight = Math.toDegrees(Math.atan2(goalRightX - shooterX, goalBackY - shooterY));

        // Calculate scoring window size
        double scoringWindow = Math.abs(angleToRight - angleToLeft);

        // If scoring window is too small, aim for center
        if (scoringWindow < MIN_SCORING_ANGLE) {
            double goalCenterX = 0.0; // AprilTag is at center of goal front
            return Math.toDegrees(Math.atan2(goalCenterX - shooterX, goalBackY - shooterY));
        }

        // Optimal aiming point: aim at the center of the available scoring window
        return (angleToLeft + angleToRight) / 2.0;
    }

    /**
     * Check if driver is providing manual input that should override alignment
     * @return true if manual override detected
     */
    private boolean isManualOverride() {
        // Check all drive-related stick inputs using GamepadEx methods
        if(m_gamepad == null) return false;
        double leftStickX = Math.abs(m_gamepad.getLeftX());
        double leftStickY = Math.abs(m_gamepad.getLeftY());
        double rightStickX = Math.abs(m_gamepad.getRightX());
        double rightStickY = Math.abs(m_gamepad.getRightY());

        // Return true if any stick exceeds threshold
        return leftStickX > STICK_THRESHOLD ||
                leftStickY > STICK_THRESHOLD ||
                rightStickX > STICK_THRESHOLD ||
                rightStickY > STICK_THRESHOLD;
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