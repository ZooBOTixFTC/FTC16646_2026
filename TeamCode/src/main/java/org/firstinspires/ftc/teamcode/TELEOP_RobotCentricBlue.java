package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Blue Robot Centric", group = "Teleop Blue")
public class TELEOP_RobotCentricBlue extends TELEOP_FieldCentric {
    @Override
    public void setSide(){
        m_robot.setBlueSide();
    }
}
