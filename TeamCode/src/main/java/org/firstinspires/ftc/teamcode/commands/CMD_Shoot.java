package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Vision;

public class CMD_Shoot extends SequentialCommandGroup {
    public CMD_Shoot(SUB_Shooter p_Shooter, SUB_ColorSensor p_colorSensor, GlobalVariables p_Variables, GlobalVariables.RobotShootState p_ShootState, SUB_Vision p_Vision) {

            if (p_colorSensor.detectColorLeft().equals(p_Vision.getNextColor())) p_Variables.setNextChamber("LEFT");
            else if (p_colorSensor.detectColorRight().equals(p_Vision.getNextColor())) p_Variables.setNextChamber("RIGHT");

        addCommands(
                    new ConditionalCommand(
                    new CMD_ShootFull(p_Shooter, p_colorSensor),
                    new CMD_ShootPattern(p_Shooter, (p_Variables.getNextChamber() == GlobalVariables.NextChamber.LEFT)),
                    ()->p_ShootState == GlobalVariables.RobotShootState.FULL
                    )
            );
    }

}
