package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.ColorConstants.ColorEnum;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShootFull extends SequentialCommandGroup {
    public CMD_ShootFull(SUB_Shooter p_shooter, SUB_ColorSensor p_colorSensor){

        addRequirements(p_shooter);
        addCommands(
                new InstantCommand(()-> p_shooter.setVelocity(0.0)),
                new CMD_GetShooterAtVelocity(p_shooter),
                new CMD_KickLeft(p_shooter),
                new CMD_GetShooterAtVelocity(p_shooter),
                new CMD_KickRight(p_shooter),
                new CMD_GetShooterAtVelocity(p_shooter),
                new ConditionalCommand(
                        new CMD_KickLeft(p_shooter),
                        new CMD_KickRight(p_shooter),
                        ()-> p_colorSensor.detectColorLeft() == ColorEnum.GREEN || p_colorSensor.detectColorLeft() == ColorEnum.PURPLE
                )

        );
    }
}
