package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Intake;

public class CMD_IntakeReverse extends SequentialCommandGroup {
    public CMD_IntakeReverse(SUB_Intake p_intake){
        addCommands(
            new InstantCommand(()-> GlobalVariables.m_intakeOn = false)
            ,new ConditionalCommand(
                new InstantCommand(()-> p_intake.setMotorPower(IntakeConstants.kIntakeOff))
                        .alongWith(new InstantCommand(()-> GlobalVariables.m_intakeReverse = false))

                ,new InstantCommand(()-> p_intake.setMotorPower(IntakeConstants.kIntakeReverse))
                        .alongWith(new InstantCommand(()-> GlobalVariables.m_intakeReverse = true))

                ,()-> GlobalVariables.m_intakeReverse
            )
        );
    }
}
