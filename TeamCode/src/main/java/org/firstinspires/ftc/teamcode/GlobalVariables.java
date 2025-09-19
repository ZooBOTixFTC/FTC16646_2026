package org.firstinspires.ftc.teamcode;

public class GlobalVariables {

     public int inPatternAlready = 0;
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

     public enum RobotShootState {
          PATTERN,
          FULL
     }

     RobotShootState m_robotShootState = RobotShootState.PATTERN;

     public RobotShootState getRobotShootState() {
          return m_robotShootState;
     }

     public void setRobotShootState(RobotShootState p_robotShootState) {
          m_robotShootState = p_robotShootState;
     }

     public boolean isRobotState(RobotShootState p_robotShootState) {
          return m_robotShootState == p_robotShootState;
     }
     public void setInPatternAlready(int alreadyInPattern) {
          inPatternAlready = alreadyInPattern;
     }

     public int getAlreadyInPattern() {
          return inPatternAlready;
     }
}
