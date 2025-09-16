package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_Shoot extends SequentialCommandGroup {
    public CMD_Shoot(SUB_Shooter p_Shooter, SUB_ColorSensor p_colorSensor, GlobalVariables.NextChamber p_NextPattern, GlobalVariables.RobotShootState p_ShootState) {
            addCommands(
                    new ConditionalCommand(
                    new CMD_ShootFull(p_Shooter, p_colorSensor),
                    new CMD_ShootPattern(p_Shooter, p_NextPattern == GlobalVariables.NextChamber.LEFT),
                    ()->p_ShootState == GlobalVariables.RobotShootState.FULL
                    )
            );
    }

}
