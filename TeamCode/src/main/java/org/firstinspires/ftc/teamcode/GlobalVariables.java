package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;

public class GlobalVariables {
    public static Pose2d m_limelightPose = new Pose2d(0, 0, 0);
    public static Pose2d m_filteredPose = new Pose2d(0, 0, 0);
    public static int m_LLNumTagsVisible = 0;
    public static boolean m_limelightConnected = false;
    public static boolean m_redSide = true;
}
