package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ColorSensor;

public class CMD_LedDefault extends CommandBase {

    private final SUB_ColorSensor m_colorSensor;
    public CMD_LedDefault(SUB_ColorSensor p_colorSensor) {
        addRequirements(p_colorSensor);
        m_colorSensor = p_colorSensor;
    }
    @Override
    public void execute () {

        if (m_colorSensor.detectColorLeft() == "Green") {
            m_colorSensor.setLEDLeftColor(Constants.ColorConstants.kGreen);
        } else if (m_colorSensor.detectColorLeft() == "Purple") {
            m_colorSensor.setLEDLeftColor(Constants.ColorConstants.kPurple);
        } else {
            m_colorSensor.setLEDLeftColor(Constants.ColorConstants.kOrange);
        }

        if (m_colorSensor.detectColorRight() == "Green") {
            m_colorSensor.setLEDRightColor(Constants.ColorConstants.kGreen);
        } else if (m_colorSensor.detectColorRight() == "Purple") {
            m_colorSensor.setLEDRightColor(Constants.ColorConstants.kPurple);
        } else m_colorSensor.setLEDRightColor(Constants.ColorConstants.kOrange);
    }
}
