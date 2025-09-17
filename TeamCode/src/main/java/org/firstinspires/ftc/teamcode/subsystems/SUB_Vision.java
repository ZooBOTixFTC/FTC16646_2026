package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.visionprocessor.AprilTagPipeline;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class SUB_Vision extends SubsystemBase {
    private OpenCvCamera webcam;
    private AprilTagPipeline aprilTagPipeline;
    private AprilTagDetection latestDetection = null;

    // Camera intrinsics (tune these for your LifeCam & resolution)
    static final double FX = 578.272;
    static final double FY = 578.272;
    static final double CX = 402.145;
    static final double CY = 221.506;

    // Physical tag size (in meters)
    static final double TAG_SIZE = 0.166;  // 16.6 cm
    private final OpMode m_OpMode;

    public SUB_Vision(OpMode p_opMode) {
        m_OpMode = p_opMode;
        int cameraMonitorViewId = m_OpMode.hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", m_OpMode.hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                m_OpMode.hardwareMap.get(WebcamName.class, "camera1"), cameraMonitorViewId);

        aprilTagPipeline = new AprilTagPipeline(TAG_SIZE, FX, FY, CX, CY) {
            @Override
            public Mat processFrame(Mat input) {
                return null;
            }
        };
        webcam.setPipeline(aprilTagPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // handle errors here
            }
        });
    }

    /** Updates and returns the latest AprilTag detection, or null if none */
    public AprilTagDetection getLatestDetection() {webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
            webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        }

        @Override
        public void onError(int errorCode) {
            // handle errors here
        }
    });
        ArrayList<AprilTagDetection> detections = aprilTagPipeline.getDetections();

        if (!detections.isEmpty()) {
            latestDetection = detections.get(0); // Take the first detected tag
        }
        return latestDetection;
    }

    /** Returns the ID of the detected AprilTag, or -1 if none */
    public int getDetectedTagID() {
        AprilTagDetection detection = getLatestDetection();
        return (detection != null) ? detection.id : -1;
    }

    public void shutdown() {
        aprilTagPipeline.release();        // frees native AprilTag detector
        webcam.stopStreaming();    // stops webcam
        webcam.closeCameraDevice(); // fully releases camera
    }

    public String decodePatternTags() {

        if (getDetectedTagID() == 21) return "G,P,P";
        else if (getDetectedTagID() == 22)  return "P,G,P";
        else if (getDetectedTagID() == 23) return "P,P,G";
        else return "Unknown";

        }

    @Override
    public void periodic() {
        m_OpMode.telemetry.addData("Detected ID",getDetectedTagID());
    }

    }






