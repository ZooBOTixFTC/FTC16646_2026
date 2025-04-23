package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.KalmanFilter;

public class SUB_Limelight extends SubsystemBase {
    private final Limelight3A m_limelight;
    private final OpMode m_opMode;
    private final SUB_Drivetrain m_drivetrain;
    private final KalmanFilter m_kalmanFilter;

    public SUB_Limelight(OpMode p_opMode, SUB_Drivetrain p_drivetrain) {
        m_drivetrain = p_drivetrain;
        m_opMode = p_opMode;
        m_limelight = m_opMode.hardwareMap.get(Limelight3A.class, "limelight");
        m_limelight.setPollRateHz(50);
        m_limelight.pipelineSwitch(0);
        m_limelight.start();

        // Initialize Kalman filter with position and velocity
        m_kalmanFilter = new KalmanFilter(0, 0, 0, 0, 0.1, 0.1, 0.1, 0.1, 1, 1, 1, 1);
    }

    public void setPipelineIndex(int index) {
        m_limelight.pipelineSwitch(index);
    }

    public Pose2d getLLPose() {
        return GlobalVariables.m_limelightPose;
    }

    public void updateDuringInit(){
        //call during init loop in auto/teleop to get vision updates before start button press
        GlobalVariables.m_limelightConnected = m_limelight.isConnected();

        m_limelight.updateRobotOrientation(m_drivetrain.getYaw());
        LLResult m_result = m_limelight.getLatestResult();

        if (m_result != null && m_result.isValid() && GlobalVariables.m_limelightConnected) {
            GlobalVariables.m_limelightPose = new Pose2d(
                    m_result.getBotpose().getPosition().x * Constants.kMetersToInches
                    ,m_result.getBotpose().getPosition().y * Constants.kMetersToInches
                    ,m_drivetrain.getYaw()
            );

            // Adjust measurement noise based on tags & speed
            double measurementNoise = GlobalVariables.m_LLNumTagsVisible > 1 ? 0.25 : 2;
            measurementNoise *= Math.min(1, m_drivetrain.getChassisSpeed() / 30);
            measurementNoise *= Math.min(1, (m_result.getBotposeAvgDist() * 39.3701) / 48);

            m_kalmanFilter.adjustMeasurementNoise(measurementNoise, measurementNoise);
            m_opMode.telemetry.addData("measurement noise", measurementNoise);

            // Correct pose using Limelight data
            m_kalmanFilter.correct(GlobalVariables.m_limelightPose.position.x, GlobalVariables.m_limelightPose.position.y);
            m_opMode.telemetry.addData("LL Pose", GlobalVariables.m_limelightPose);
            // Get filtered pose
            double[] filteredState = m_kalmanFilter.getFilteredState();
            GlobalVariables.m_filteredPose = new Pose2d(filteredState[0], filteredState[1], m_drivetrain.getYaw());
            m_drivetrain.setLocalizerPose(GlobalVariables.m_filteredPose);

            m_opMode.telemetry.addData("filtered pose", GlobalVariables.m_filteredPose);
        }
    }

    @Override
    public void periodic() {
        GlobalVariables.m_limelightConnected = m_limelight.isConnected();

        m_limelight.updateRobotOrientation(m_drivetrain.getYaw());
        LLResult m_result = m_limelight.getLatestResult();

        if (m_result != null && m_result.isValid() && GlobalVariables.m_limelightConnected) {
            GlobalVariables.m_limelightPose = new Pose2d(
                m_result.getBotpose().getPosition().x * Constants.kMetersToInches
                ,m_result.getBotpose().getPosition().y * Constants.kMetersToInches
                ,m_drivetrain.getYaw()
            );

            // Adjust measurement noise based on tags & speed
            double measurementNoise = GlobalVariables.m_LLNumTagsVisible > 1 ? 0.25 : 2;
            measurementNoise *= Math.min(1, m_drivetrain.getChassisSpeed() / 30);
            measurementNoise *= Math.min(1, (m_result.getBotposeAvgDist() * 39.3701) / 48);

            m_kalmanFilter.adjustMeasurementNoise(measurementNoise, measurementNoise);
            m_opMode.telemetry.addData("measurement noise", measurementNoise);

            // Correct pose using Limelight data
            m_kalmanFilter.correct(GlobalVariables.m_limelightPose.position.x, GlobalVariables.m_limelightPose.position.y);
            m_opMode.telemetry.addData("LL Pose", GlobalVariables.m_limelightPose);
        }

        // Retrieve encoder data for prediction
        double encoderX = m_drivetrain.getDrivePose().position.x;
        double encoderY = m_drivetrain.getDrivePose().position.y;
        double encoderVX = m_drivetrain.getDriveVelocity().x;
        double encoderVY = m_drivetrain.getDriveVelocity().y;

        // Predict the next state
        m_kalmanFilter.predict(encoderX, encoderY, encoderVX, encoderVY);

        // Get filtered pose
        double[] filteredState = m_kalmanFilter.getFilteredState();
        GlobalVariables.m_filteredPose = new Pose2d(filteredState[0], filteredState[1], m_drivetrain.getYaw());

        m_opMode.telemetry.addData("filtered pose", GlobalVariables.m_filteredPose);
    }
}
