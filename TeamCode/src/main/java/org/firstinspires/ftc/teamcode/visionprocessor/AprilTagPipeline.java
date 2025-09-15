package org.firstinspires.ftc.teamcode.visionprocessor;

import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class AprilTagPipeline extends OpenCvPipeline
{
    private long nativeApriltagPtr;  // handle to native detector
    private static ArrayList<AprilTagDetection> detections = new ArrayList<>();

    // Calibration
    private double tagsize, fx, fy, cx, cy;

    public AprilTagPipeline(double tagsize, double fx, double fy, double cx, double cy)
    {
        this.tagsize = tagsize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        // Create the native detector (tag36h11 family is default for FTC)
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(
                AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Convert to grayscale (AprilTag expects grayscale input)
        Mat gray = new Mat();
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);

        // Run detection
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(
                nativeApriltagPtr, gray, tagsize, fx, fy, cx, cy);

        // Free grayscale buffer
        gray.release();

        return input; // return original frame (you can draw overlays later)
    }

    public static ArrayList<AprilTagDetection> getDetections()
    {
        return detections;
    }

    public void release()
    {
        AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
    }
}
