package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Turntable;

public class CMD_ShootAll extends SequentialCommandGroup {

    public CMD_ShootAll(SUB_Shooter p_Shooter, SUB_Turntable p_Turntable, GlobalVariables p_Variables) {
        addCommands(
                new CMD_Shoot(p_Shooter,p_Turntable,p_Variables)
                ,new CMD_Shoot(p_Shooter,p_Turntable,p_Variables)
                ,new CMD_Shoot(p_Shooter,p_Turntable,p_Variables)
        );
    }
}
