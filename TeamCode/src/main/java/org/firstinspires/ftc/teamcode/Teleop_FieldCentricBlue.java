package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Blue", group = "Teleop blue")
public class Teleop_FieldCentricBlue extends Teleop_FieldCentricRed{
    @Override
    public void setSide(){
        GlobalVariables.m_redSide = false;
    }
}
