package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

/**
 * A subsystem that uses the {@link SampleMecanumDrive} class.
 * This periodically calls {@link SampleMecanumDrive#update()} which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class MecanumDriveSubsystem extends com.arcrobotics.ftclib.command.SubsystemBase {

    private final SampleMecanumDrive drive;
    private boolean fieldCentric = true;
    private final OpMode m_opMode;

    public MecanumDriveSubsystem(SampleMecanumDrive drive, OpMode p_opMode) {
        this.drive = drive;
        m_opMode = p_opMode;
    }

    public void setMode(DcMotor.RunMode mode) {
        drive.setMode(mode);
    }

    public void setMotorTargetPositions(int pos, int pos1, int pos2, int pos3){
        drive.setMotorTargetPositions(pos, pos1, pos2, pos3);
    }

    public void setMotorPowers(double v, double v1, double v2, double v3){
        drive.setMotorPowers(v, v1, v2, v3);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(mode, coefficients);
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void update() {
        drive.update();
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public void drive(double leftY, double leftX, double rightX) {
        Vector2d input = new Vector2d(-leftY, -leftX);

        drive.setWeightedDrivePower(
            new Pose2d(
                input.getX(),
                input.getY(),
                -rightX
            )
        );
    }

    public void drive(double leftY, double leftX, double rightX, double driverOffsetAngle) {
        Pose2d poseEstimate = getPoseEstimate();

        Vector2d input = new Vector2d(-leftY, -leftX).rotated(
            fieldCentric ? -poseEstimate.getHeading() + Math.toRadians(driverOffsetAngle) : 0
        );

        drive.setWeightedDrivePower(
            new Pose2d(
                input.getX(),
                input.getY(),
                -rightX
            )
        );
    }

    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void turn(double radians) {
        drive.turnAsync(radians);
    }

    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    public List<Double> getWheelPositions(){
        return drive.getWheelPositions();
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }

    public Localizer getLocalizer() {
        return drive.getLocalizer();
    }

    public void setFieldCentric(boolean p_fieldCentric) {
        fieldCentric = p_fieldCentric;
    }
}
