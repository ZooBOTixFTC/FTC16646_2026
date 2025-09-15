package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.ColorConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ColorSensor;

public class CMD_LedDefault extends CommandBase {

    private final SUB_ColorSensor m_colorSensor;
    public CMD_LedDefault(SUB_ColorSensor p_colorSensor) {
        addRequirements(p_colorSensor);
        m_colorSensor = p_colorSensor;
    }
    @Override
    public void execute () {
        switch (m_colorSensor.detectColorLeft()){
            case GREEN:
                m_colorSensor.setLEDLeftColor(ColorConstants.kGreen);
                break;
            case PURPLE:
                m_colorSensor.setLEDLeftColor(ColorConstants.kPurple);
                break;
            default:
                break;
        }

        switch (m_colorSensor.detectColorRight()){
            case GREEN:
                m_colorSensor.setLEDRightColor(ColorConstants.kGreen);
                break;
            case PURPLE:
                m_colorSensor.setLEDRightColor(ColorConstants.kPurple);
                break;
            default:
                break;
        }
    }
}
