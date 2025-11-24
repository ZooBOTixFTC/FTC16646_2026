package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Intake;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Kicker;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_Shoot extends SequentialCommandGroup {
    public CMD_Shoot(SUB_Shooter p_shooter, SUB_Kicker p_kicker, SUB_Intake p_intake){
        addRequirements(p_shooter);

        addCommands(
                new CMD_GetShooterAtVelocity(p_shooter)
                ,new CMD_Kick(p_kicker)
                ,new WaitCommand(500)
                ,new CMD_GetShooterAtVelocity(p_shooter)
                ,new InstantCommand(()-> p_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
                ,new WaitCommand(150)
                ,new CMD_Kick(p_kicker)
                ,new WaitCommand(750)
        );
    }
}
