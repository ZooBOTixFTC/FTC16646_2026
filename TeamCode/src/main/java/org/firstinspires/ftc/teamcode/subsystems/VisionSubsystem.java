package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.visionprocessor.AprilTagPipeline;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.apriltag.AprilTagDetection;

import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class VisionSubsystem {
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

    public VisionSubsystem(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "camera1"), cameraMonitorViewId);

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
}
