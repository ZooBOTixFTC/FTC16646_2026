package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
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
    private static final int STABLE_COUNT = 5; // Number of consecutive stable cycles required to finish
    private static final int MAX_LOST_FRAMES = 15; // Allow brief target loss before giving up

    // Manual override parameters - TUNE THESE FOR DRIVER COMFORT
    private static final double STICK_THRESHOLD = 0.3;     // Minimum stick movement to trigger override (increased to prevent false triggers)

    // =====================================================================

    private final MecanumDriveSubsystem m_drive;
    private final SUB_Vision m_vision;
    private final GlobalVariables m_variables;
    private final GamepadEx m_gamepad;

    private boolean isFinished;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private int stableCount = 0;
    private int lostTargetCount = 0;

    public CMD_AlignTarget(MecanumDriveSubsystem drive, SUB_Vision vision, GlobalVariables variables, GamepadEx gamepad) {
        this.m_drive = drive;
        this.m_vision = vision;
        this.m_variables = variables;
        this.m_gamepad = gamepad;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        isFinished = false;
        lastError = 0.0;
        lastTime = System.currentTimeMillis() / 1000.0;
        stableCount = 0;
        lostTargetCount = 0;
    }

    @Override
    public void execute() {
        // Check for manual override first
        if (isManualOverride()) {
            m_drive.drive(0.0, 0.0, 0.0);
            isFinished = true;
            return;
        }

        // Get fresh AprilTag detections every cycle
        int targetId = m_variables.m_red ? 24 : 20;
        AprilTagDetection currentDetection = null;

        // Find the target AprilTag in current detections
        for (AprilTagDetection detection : m_vision.getDetections()) {
            if (detection.id == targetId) {
                currentDetection = detection;
                break;
            }
        }

        // If no target tag detected, handle gracefully
        if (currentDetection == null) {
            lostTargetCount++;
            
            // Only give up after losing target for multiple consecutive frames
            if (lostTargetCount > MAX_LOST_FRAMES) {
                m_drive.drive(0.0, 0.0, 0.0);
                isFinished = true;
            } else {
                // Brief loss - hold position and wait for target to reappear
                m_drive.drive(0.0, 0.0, 0.0);
            }
            return;
        }
        
        // Target found - reset lost counter
        lostTargetCount = 0;

        // Calculate optimal aiming angle (may be different from AprilTag bearing)
        double bearing = calculateOptimalAiming(currentDetection);
        double currentTime = System.currentTimeMillis() / 1000.0;
        double deltaTime = currentTime - lastTime;

        // Control parameters
        double tolerance = 2.0; // degrees - target tolerance
        double deadband = 1.25; // degrees - deadband to prevent oscillation
        double kP = 0.015;      // Proportional gain (reduced to prevent overshoot)
        double kD = 0.018;      // Derivative gain to reduce oscillation (increased for damping)

        // Check if we're within the deadband (smaller than tolerance)
        if (Math.abs(bearing) < deadband) {
            m_drive.drive(0.0, 0.0, 0.0);
            stableCount++;

            // Require stability for several cycles before finishing
            if (stableCount >= STABLE_COUNT) {
                isFinished = true;
            }

            lastError = bearing;
            lastTime = currentTime;
            return;
        } else {
            stableCount = 0; // Reset stability counter if we move out of deadband
        }

        // Calculate derivative term
        double derivative = 0.0;
        if (deltaTime > 0) {
            derivative = (bearing - lastError) / deltaTime;
        }

        // PD control calculation
        double turnPower = -(kP * bearing + kD * derivative);

        // Apply power limits
        double maxTurnPower = 0.20; // Maximum turn power (reduced to prevent overshoot)
        double minTurnPower = 0.05; // Minimum power threshold (lowered for smoother approach)

        turnPower = Math.max(-maxTurnPower, Math.min(maxTurnPower, turnPower));

        // Apply minimum power threshold only for larger errors (beyond tolerance)
        if (Math.abs(bearing) > tolerance && Math.abs(turnPower) > 0 && Math.abs(turnPower) < minTurnPower) {
            turnPower = Math.copySign(minTurnPower, turnPower);
        }

        // Store values for next cycle
        lastError = bearing;
        lastTime = currentTime;

        m_drive.drive(0.0, 0.0, turnPower);
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
        double optimalAngle = (angleToLeft + angleToRight) / 2.0;

        return optimalAngle;
    }

    /**
     * Check if driver is providing manual input that should override alignment
     * @return true if manual override detected
     */
    private boolean isManualOverride() {
        // Check all drive-related stick inputs using GamepadEx methods
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
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}