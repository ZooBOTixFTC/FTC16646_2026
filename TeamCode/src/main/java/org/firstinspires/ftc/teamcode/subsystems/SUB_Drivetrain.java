package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

public class SUB_Drivetrain extends SubsystemBase {
    private final MecanumDrive m_drivebase;
    private final OpMode m_opMode;
    private boolean m_fieldCentric;
    private Pose2d previousPose;
    private double lastUpdateTime;

    public SUB_Drivetrain(OpMode p_opMode, MecanumDrive p_drivebase, boolean p_fieldCentric){
        m_opMode = p_opMode;
        m_drivebase = p_drivebase;
        m_fieldCentric = p_fieldCentric;
        lastUpdateTime = System.nanoTime();
    }

    public double getYaw(){
        return m_drivebase.lazyImu.get().getRobotYawPitchRollAngles().getYaw();
    }

    public Pose2d getDrivePose(){
        return m_drivebase.localizer.getPose();
    }

    public void drive(double xSpeed, double ySpeed, double turnSpeed){
        //clamp drivespeeds, constraining them to -1 to 1
        xSpeed = MathUtils.clamp(xSpeed, -1, 1);
        ySpeed = MathUtils.clamp(ySpeed, -1, 1);
        turnSpeed = MathUtils.clamp(turnSpeed, -1, 1);

        if (m_fieldCentric) {
            double yawRad = getYaw() * Math.PI / 180; // Convert yaw to radians

            // Use temporary variables to avoid overwriting xSpeed before calculating ySpeed
            double tempX = xSpeed * Math.cos(yawRad) - ySpeed * Math.sin(yawRad);
            double tempY = xSpeed * Math.sin(yawRad) + ySpeed * Math.cos(yawRad);

            xSpeed = tempX; // Update xSpeed with transformed value
            ySpeed = tempY; // Update ySpeed with transformed value
        }

        m_drivebase.setDrivePowers(new PoseVelocity2d(new Vector2d(xSpeed, ySpeed), turnSpeed));
    }

    public double getChassisSpeed() {
        //average the absolute value of all 4 drive motors
        double vel = (Math.abs(m_drivebase.leftFront.getVelocity()) + Math.abs(m_drivebase.rightFront.getVelocity())
                + Math.abs(m_drivebase.leftBack.getVelocity()) + Math.abs(m_drivebase.rightBack.getVelocity())) / 4;
        double wheelCircumference = 3 * Math.PI;//3 in wheels
        return (vel * wheelCircumference) / 537.7;//ticks per rev for a GoBilda 312 rpm motor
    }

    public Vector2d getDriveVelocity() {
        double currentTime = System.nanoTime();
        double dt = (currentTime - lastUpdateTime) / 1e9; // Convert nanoseconds to seconds
        lastUpdateTime = currentTime;

        Pose2d currentPose = getDrivePose();

        if (previousPose != null && dt > 0) {
            double velocityX = (currentPose.position.x - previousPose.position.x) / dt;
            double velocityY = (currentPose.position.y - previousPose.position.y) / dt;
            previousPose = currentPose;
            return new Vector2d(velocityX, velocityY);
        }

        previousPose = currentPose;
        return new Vector2d(0, 0); // Default to zero if first calculation
    }

    public Action runTrajectory(Trajectory p_trajectory){
        return m_drivebase.new FollowTrajectoryAction(new TimeTrajectory(p_trajectory));
    }

    public TrajectoryActionBuilder getActionBuilder(){
        return m_drivebase.actionBuilder(getDrivePose());
    }

    public void setFieldCentric(boolean p_fieldCentricEnabled){
        m_fieldCentric = p_fieldCentricEnabled;
    }

    public void setLocalizerPose(Pose2d pose){
        m_drivebase.localizer.setPose(pose);
    }

    @Override
    public void periodic(){
        m_drivebase.localizer.update();
        //if no limelight connected, fallback to use only drive encoders instead of filtered pose
        if(GlobalVariables.m_limelightConnected){
            //update pose with kalman filter one from LL
            m_drivebase.localizer.setPose(GlobalVariables.m_filteredPose);
        }
        //update drive encoder estimated pose and display it on the ds screen
        m_opMode.telemetry.addData("drive pose", getDrivePose());
    }
}
