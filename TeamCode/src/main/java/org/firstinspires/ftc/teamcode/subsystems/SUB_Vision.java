package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.GlobalVariables.patternTypes;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class SUB_Vision extends SubsystemBase {
    private final OpMode m_opMode;
    private final AprilTagProcessor m_aprilTagProcessor;
    private final VisionPortal m_visionPortal;
    private final GlobalVariables m_variables;
    private List<AprilTagDetection> m_detections;

    public SUB_Vision(OpMode p_opMode, GlobalVariables p_variables) {
        m_opMode = p_opMode;
        m_variables = p_variables;

        m_aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        m_aprilTagProcessor.setDecimation(1);

        m_visionPortal = new VisionPortal.Builder()
                .setCamera(m_opMode.hardwareMap.get(WebcamName.class, "camera1"))
                .addProcessor(m_aprilTagProcessor)
                .build();

        m_visionPortal.setProcessorEnabled(m_aprilTagProcessor, true);
    }

    public void stream(boolean stream){
        if(stream){
            m_visionPortal.resumeStreaming();
        }else{
            m_visionPortal.stopStreaming();
        }
    }

    public void readPattern(){
        for (AprilTagDetection detection : getDetections()){
            switch (detection.id){
                case 21:
                    m_variables.setPatternType(patternTypes.GPP);
                    break;
                case 22:
                    m_variables.setPatternType(patternTypes.PGP);
                    break;
                case 23:
                    m_variables.setPatternType(patternTypes.PPG);
                    break;
                default:
                    break;
            }
        }
    }

    public List<AprilTagDetection> getDetections(){
        return m_detections;
    }

    public double getDistToTag(){
        double dist = 0;
        if(!getDetections().isEmpty()){
            for(AprilTagDetection detection: getDetections()){
                if(detection.id == 20 && !GlobalVariables.m_red){
                    dist = detection.ftcPose.range;
                }
                if(detection.id == 24 && GlobalVariables.m_red){
                    dist = detection.ftcPose.range;
                }
            }
        }

        return dist;
    }

    @Override
    public void periodic(){
        try {
            m_detections = m_aprilTagProcessor.getDetections();

            if (m_detections != null) {
                m_opMode.telemetry.addData("# AprilTags detected", m_detections.size());
                for (AprilTagDetection detection : m_detections) {
                    m_opMode.telemetry.addData("ID", detection.id);
                    m_opMode.telemetry.addData("distance", detection.ftcPose.range);
                    m_opMode.telemetry.addData("yaw", detection.ftcPose.yaw);
                    m_opMode.telemetry.addData("bearing", detection.ftcPose.bearing);
                }
            }

            m_opMode.telemetry.addData("vision portal status", m_visionPortal.getCameraState());
            m_opMode.telemetry.addData("fps", m_visionPortal.getFps());
        } catch (Exception e) {
            m_opMode.telemetry.addData("vision error", e.getStackTrace());
        }
    }
}