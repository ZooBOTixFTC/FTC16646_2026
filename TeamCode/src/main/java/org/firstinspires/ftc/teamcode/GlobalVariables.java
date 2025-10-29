package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class GlobalVariables {

     public static Pose2d m_autoEndPose = new Pose2d();
     public static boolean m_intakeOn, m_intakeReverse = false;
     public static boolean m_kickerHomed = true;
     public static boolean m_red;

     public enum patternTypes {
          GPP,
          PGP,
          PPG
     }

     public static patternTypes m_patternType = patternTypes.PGP;
}
