package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class GlobalVariables {
     public enum RobotState {
          HOME
     }

     RobotState m_robotState = RobotState.HOME;

     public RobotState getRobotState() {
          return m_robotState;
     }

     public void setRobotState(RobotState p_robotState) {
          m_robotState = p_robotState;
     }

     public boolean isRobotState(RobotState p_robotState) {
          return m_robotState == p_robotState;
     }
}
