package org.firstinspires.ftc.teamcode;

public class GlobalVariables {
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
}
