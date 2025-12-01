package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Turntable;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Vision;

public class CMD_ShootAll extends SequentialCommandGroup {
    public CMD_ShootAll(SUB_Shooter p_shooter, SUB_Turntable p_turntable, MecanumDriveSubsystem p_drive, SUB_Vision p_vision) {
        addCommands(
            new CMD_AlignTarget(p_drive, p_vision)
            ,new CMD_Shoot(p_shooter, p_turntable)
            ,new InstantCommand(p_turntable::rotateRight)
            ,new CMD_Shoot(p_shooter,p_turntable)
            ,new InstantCommand(p_turntable::rotateRight)
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
