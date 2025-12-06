package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.GlobalVariables;

public class SUB_Limelight extends SubsystemBase {
    private final Limelight3A m_limelight;
    private final OpMode m_opMode;

    public SUB_Limelight(OpMode p_opMode){
        m_opMode = p_opMode;

        m_limelight = m_opMode.hardwareMap.get(Limelight3A.class, "limelight");

        m_limelight.pipelineSwitch(0);
        m_limelight.setPollRateHz(45);
        m_limelight.start();
    }

    public Vector2d getPose(){
            for(LLResultTypes.FiducialResult fiducialResult : getLatestResult().getFiducialResults()){
                if(fiducialResult.getFiducialId() == (GlobalVariables.m_red ? 24 : 20)){
                   return new Vector2d(
                       metersToInches(fiducialResult.getRobotPoseFieldSpace().getPosition().x),
                       metersToInches(fiducialResult.getRobotPoseFieldSpace().getPosition().y));
            }
        }
        return new Vector2d();
    }

    public LLResult getLatestResult(){
        return m_limelight.getLatestResult();
    }

    public boolean isResultValid(){
        return getLatestResult().isValid();
    }

    public double getDistToTag(){
        if(isResultValid()){
            for(LLResultTypes.FiducialResult fiducialResult : getLatestResult().getFiducialResults()){
                if(fiducialResult.getFiducialId() == (GlobalVariables.m_red ? 24 : 20)){
                    Position pos = fiducialResult.getRobotPoseTargetSpace().getPosition();
                    double dist = metersToInches(Math.sqrt(
                        pos.x * pos.x +
                        pos.y * pos.y +
                        pos.z * pos.z
                    ));

                    GlobalVariables.m_distToTag = dist;
                    return dist;
                }
            }
        }
        return GlobalVariables.m_distToTag;
    }

    public double getBearing(){
        if(isResultValid()){
            for(LLResultTypes.FiducialResult fiducialResult : getLatestResult().getFiducialResults()){
                if(fiducialResult.getFiducialId() == (GlobalVariables.m_red ? 24 : 20)){
                    return Math.atan2(
                        fiducialResult.getRobotPoseTargetSpace().getPosition().y,
                        fiducialResult.getRobotPoseTargetSpace().getPosition().x);
                }
            }
        }
        return 0;
    }

    @Override
    public void periodic(){

        LLStatus status = m_limelight.getStatus();

        m_opMode.telemetry.addData("LL", "Temp: %.1f, CPU: %.1f%%, FPS: %.1f",
                status.getTemp(), status.getCpu(),(int)status.getFps());

        if(isResultValid()){
            for(LLResultTypes.FiducialResult fiducialResult : getLatestResult().getFiducialResults()){
                if(fiducialResult.getFiducialId() == (GlobalVariables.m_red ? 24 : 20)) {
                    m_opMode.telemetry.addData("ID", fiducialResult.getFiducialId());

                    m_opMode.telemetry.addData("pose", "X: %.1f, Y: %.1f",
                            getPose().getX(), getPose().getY());

                    m_opMode.telemetry.addData("dist", getDistToTag());
                    m_opMode.telemetry.addData("bearing", Math.toDegrees(getBearing()));
                }
            }
        }
    }

    private double metersToInches(double meters){
        return meters * 39.3701;
    }
}
