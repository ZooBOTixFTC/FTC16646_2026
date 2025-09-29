package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Intake;

public class CMD_IntakeToggle extends SequentialCommandGroup {
    public CMD_IntakeToggle(SUB_Intake p_intake){
        addCommands(
            new ConditionalCommand(
                new InstantCommand(()-> p_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
                    .alongWith(new InstantCommand(()-> GlobalVariables.m_intakeOn = false))

                ,new InstantCommand(()-> p_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
                    .alongWith(new InstantCommand(()-> GlobalVariables.m_intakeOn = true))

                ,()-> GlobalVariables.m_intakeOn
            )
        );
    }
}
