package org.firstinspires.ftc.teamcode;

public class GlobalVariables {
     public boolean m_red;

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
