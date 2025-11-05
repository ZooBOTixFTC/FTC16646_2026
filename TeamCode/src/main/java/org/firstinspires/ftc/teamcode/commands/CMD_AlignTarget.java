package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.AutoAlignConstants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


public class CMD_AlignTarget extends CommandBase {
    // Goal geometry (in inches) - MEASURE AND ADJUST THESE VALUES
    private static final double GOAL_WIDTH = 27.0;           // Width of goal opening (inches)
    private static final double GOAL_DEPTH = 18.0;          // Distance from AprilTag to back corner (inches)
    private static final double SHOOTER_OFFSET_X = 0.0;     // Shooter position relative to robot center (inches)
    private static final double SHOOTER_OFFSET_Y = 0.0;     // Shooter distance from robot center forward (inches)

    // Optimization parameters - TUNE THESE FOR BEST PERFORMANCE
    private static final double MIN_SCORING_ANGLE = 5.0;   // Minimum acceptable scoring angle (degrees)
    private static final double SAFETY_MARGIN = 2.0;       // Safety margin from goal edges (inches)
    private static final boolean USE_OPTIMAL_AIMING = true; // Enable/disable optimal aiming

    private static final double STICK_THRESHOLD = 0.3;     // Minimum stick movement to trigger override (increased to prevent false triggers)


    private final MecanumDriveSubsystem m_drive;
    private final SUB_Vision m_vision;
    private final GamepadEx m_gamepad;
    private final ElapsedTime m_time = new ElapsedTime();

    private boolean isFinished;
    private int m_targetTicks;

    public CMD_AlignTarget(MecanumDriveSubsystem drive, SUB_Vision vision, GamepadEx gamepad) {
        this.m_drive = drive;
        this.m_vision = vision;
        this.m_gamepad = gamepad;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_time.reset();
        isFinished = false;

        //if close to tag or no tags in view, cancel
        if(GlobalVariables.m_distToTag < AutoAlignConstants.kDistanceThreshold ||
                m_vision.getDetections().isEmpty()) cancel();

        Pose2d m_drivePose = m_drive.getPoseEstimate();

        //reset encoders
        m_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_drive.setMotorTargetPositions(0, 0, 0, 0);
        m_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_drive.setMotorPowers(1, 1, 1, 1);
        m_drive.setPoseEstimate(m_drivePose);

        AprilTagDetection m_detection = null;

        //check all detections in view and make sure ID matches alliance color
        for (AprilTagDetection detection : m_vision.getDetections()){
            if(detection.id == (GlobalVariables.m_red ? 24 : 20)){
                m_detection = detection;
                break;
            }
        }

        //if no id's in view or not the correct ID, cancel
        if (m_detection == null) cancel();

        //calculate angle to target and set target pos
        double targetAng = calculateOptimalAiming(m_detection);
        m_targetTicks = (int) (targetAng * AutoAlignConstants.kTicksPerDeg);

        m_drive.setMotorTargetPositions(m_targetTicks, m_targetTicks, -m_targetTicks, -m_targetTicks);
    }

    @Override
    public void execute() {
        //update localizer
        m_drive.update();

        //track number of wheels in the correct position and end once all are in position
        double wheelsInPos = 0;

        for(double pos : m_drive.getWheelPositions()){
            if(Math.abs(pos - m_targetTicks) < 10) wheelsInPos ++;
        }

        if (wheelsInPos == 4) isFinished = true;
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
        m_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public boolean isFinished() {
        //if in position, driver wiggles joysticks, or timeout, end command
        return isFinished || isManualOverride() || m_time.seconds() > 5;
    }
}