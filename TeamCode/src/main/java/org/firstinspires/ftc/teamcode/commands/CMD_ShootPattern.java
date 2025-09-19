package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.ColorConstants.ColorEnum;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShootPattern extends CommandBase {

    private final GlobalVariables m_variables;
    private final SUB_Shooter m_shooter;
    private final SUB_ColorSensors m_colorSensors;
    private final RobotContainer m_robot;
    private String nextColor;

    public CMD_ShootPattern(GlobalVariables p_variables, SUB_Shooter p_shooter, SUB_ColorSensors p_colorSensors, RobotContainer p_robot) {
        m_shooter = p_shooter;
        m_variables = p_variables;
        m_colorSensors = p_colorSensors;
        m_robot = p_robot;
    }

    @Override
    public void initialize() {
        //
        for (int i = 0; i < 3; i++) {
            ColorEnum nextColor = ColorEnum.PURPLE;

            switch (GlobalVariables.m_patternType){
                case GPP:
                    nextColor = Constants.ColorConstants.patterns[1][i];
                    break;
                case PGP:
                    nextColor = Constants.ColorConstants.patterns[2][i];
                    break;
                case PPG:
                    nextColor = Constants.ColorConstants.patterns[3][i];
                    break;
            }

            if(m_colorSensors.detectColorLeft() == nextColor){
                m_robot.schedule(new CMD_ShootLeft(m_shooter));
            }else{
                m_robot.schedule(new CMD_ShootRight(m_shooter));
            }
        }
    }
}
