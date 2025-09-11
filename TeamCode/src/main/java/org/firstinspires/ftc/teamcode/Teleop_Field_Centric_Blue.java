package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Blue", group = "Teleop Blue")
public class Teleop_Field_Centric_Blue extends Teleop_Field_Centric{
    @Override
    public void setSide(){
        m_robot.setBlueSide();
    }
}
