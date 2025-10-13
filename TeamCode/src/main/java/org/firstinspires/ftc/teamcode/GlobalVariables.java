package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class GlobalVariables {
     public static Pose2d m_autoEndPose = new Pose2d();
     public static boolean m_intakeOn = false;
     public static boolean m_intakeReverse = false;

     public boolean m_red = true;

     public enum patternTypes {
          GPP,
          PGP,
          PPG
     }
     public static patternTypes m_patternType = patternTypes.PGP;

     public void setPatternType(patternTypes p_patternType) {
          m_patternType = p_patternType;
     }

     public patternTypes getPatternType() {
          return m_patternType;
     }
}
