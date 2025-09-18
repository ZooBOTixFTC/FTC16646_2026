package org.firstinspires.ftc.teamcode;

public class GlobalVariables {

     public static int inPatternAlready = 0;
     public enum RobotShootState {
          PATTERN,
          FULL
     }

     public enum NextChamber {
          LEFT,
          RIGHT,
          UNKNOWN
     }

     RobotShootState m_robotShootState = RobotShootState.PATTERN;
     NextChamber m_NextChamber = NextChamber.UNKNOWN;

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
     public void setNextChamber(String side) {
          if (side.equals("left")) m_NextChamber = NextChamber.LEFT;
          else if(side.equals("right")) m_NextChamber = NextChamber.RIGHT;
          else m_NextChamber = NextChamber.UNKNOWN;
     }
     public NextChamber getNextChamber() {
          return m_NextChamber;
     }
}
