package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.visionprocessor.AprilTagPipeline;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp(name="AprilTag Detection Test", group="Test")
public class AprilTagDetectionTest extends LinearOpMode
{
    private VisionSubsystem vision;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialize the vision subsystem (opens webcam + sets pipeline)
        vision = new VisionSubsystem(hardwareMap);

        telemetry.addLine("AprilTag Vision Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            ArrayList<AprilTagDetection> detections = AprilTagPipeline.getDetections();

            if (detections.isEmpty())
            {
                telemetry.addLine("No tags detected");
            }
            else
            {
                telemetry.addData("Detected Count", detections.size());

                for (AprilTagDetection tag : detections)
                {
                    telemetry.addLine("-------------------");
                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("Hamming", tag.hamming);

                    telemetry.addData("X (m)", "%.2f", tag.pose.x);
                    telemetry.addData("Y (m)", "%.2f", tag.pose.y);
                    telemetry.addData("Z (m)", "%.2f", tag.pose.z);
                }
            }

            telemetry.update();

            // Don’t burn CPU — small pause
            sleep(20);
        }

        // Shutdown camera when OpMode ends
        vision.shutdown();
    }
}
