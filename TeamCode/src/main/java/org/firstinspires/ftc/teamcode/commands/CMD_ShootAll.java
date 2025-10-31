package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Turntable;

public class CMD_ShootAll extends SequentialCommandGroup {
    public CMD_ShootAll(SUB_Shooter p_shooter, SUB_Turntable p_turntable) {
        addCommands(
            new CMD_Shoot(p_shooter, p_turntable)
            ,new CMD_Shoot(p_shooter,p_turntable)
            ,new CMD_Shoot(p_shooter,p_turntable)
        );
    }

    public CMD_ShootAll(SUB_Shooter p_shooter, SUB_Turntable p_turntable, double p_targetVel) {
        addCommands(
                new CMD_Shoot(p_shooter,p_turntable, p_targetVel)
                ,new CMD_Shoot(p_shooter,p_turntable, p_targetVel)
                ,new CMD_Shoot(p_shooter,p_turntable, p_targetVel)
        );
    }
}
