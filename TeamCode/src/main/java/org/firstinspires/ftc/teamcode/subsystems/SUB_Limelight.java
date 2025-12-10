package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.util.MathUtils;

public class SUB_Limelight extends SubsystemBase {
    private final Limelight3A m_limelight;
    private final OpMode m_opMode;

    public SUB_Limelight(OpMode p_opMode){
        m_opMode = p_opMode;

        m_limelight = m_opMode.hardwareMap.get(Limelight3A.class, "limelight");

        m_limelight.pipelineSwitch(0);
        m_limelight.setPollRateHz(80);
        m_limelight.start();
    }

    public Pose2d getPose(AngleUnit angleUnit){
        Pose3D pose = getLatestResult().getBotpose();
        double heading = Angle.normDelta(pose.getOrientation().getYaw(AngleUnit.RADIANS) + Math.PI);
        if(angleUnit == AngleUnit.DEGREES) heading = Math.toDegrees(heading);
        return new Pose2d(-MathUtils.metersToInches(pose.getPosition().x), -MathUtils.metersToInches(pose.getPosition().y), heading);
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
                    double dist = MathUtils.metersToInches(Math.sqrt(
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
                status.getTemp(), status.getCpu(), status.getFps());

        if(isResultValid()){
            for(LLResultTypes.FiducialResult fiducialResult : getLatestResult().getFiducialResults()){
                if(fiducialResult.getFiducialId() == (GlobalVariables.m_red ? 24 : 20)) {
                    m_opMode.telemetry.addData("ID", fiducialResult.getFiducialId());

                    Pose2d pose = getPose(AngleUnit.DEGREES);
                    m_opMode.telemetry.addData("pose", "X: %.1f, Y: %.1f, H: %.1f",
                            pose.getX(), pose.getY(), pose.getHeading());

                    m_opMode.telemetry.addData("dist", getDistToTag());
                    m_opMode.telemetry.addData("bearing", Math.toDegrees(getBearing()));
                }
            }
        }
    }
}
