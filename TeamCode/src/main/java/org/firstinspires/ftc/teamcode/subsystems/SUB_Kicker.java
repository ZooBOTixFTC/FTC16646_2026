package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class SUB_Kicker extends SubsystemBase {
    private final OpMode m_opMode;

    private final Servo m_leftKickerServo;
    private final Servo m_rightKickerServo;

    public SUB_Kicker(OpMode p_opmode) {
        m_opMode = p_opmode;

        m_leftKickerServo = m_opMode.hardwareMap.get(Servo.class, "leftKickerServo");
        m_rightKickerServo = m_opMode.hardwareMap.get(Servo.class, "rightKickerServo");

        m_leftKickerServo.setDirection(Servo.Direction.FORWARD);
        m_rightKickerServo.setDirection(Servo.Direction.REVERSE);
    }

    public void kick(){
        kickLeft();
        kickRight();
    }

    public void kickLeft(){
        m_leftKickerServo.setPosition(Constants.KickConstants.kKickLeft);
    }

    public void kickRight(){
        m_rightKickerServo.setPosition(Constants.KickConstants.kKickRight);
    }

    public void home(){
        homeLeft();
        homeRight();
    }

    public void homeLeft(){
        m_leftKickerServo.setPosition(Constants.KickConstants.kHome);
    }

    public void homeRight(){
        m_rightKickerServo.setPosition(Constants.KickConstants.kHome);
    }

    public void level(){
        levelLeft();
        levelRight();
    }

    public void levelLeft(){
        m_leftKickerServo.setPosition(Constants.KickConstants.kLevelLeft);
    }

    public void levelRight(){
        m_rightKickerServo.setPosition(Constants.KickConstants.kLevelRight);
    }
}
