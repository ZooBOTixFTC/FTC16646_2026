package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Intake;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Turntable;

public class CMD_IntakeToggle extends SequentialCommandGroup {
    public CMD_IntakeToggle(SUB_Intake p_intake, SUB_Turntable p_turntable){
                addCommands(
            new ConditionalCommand(
                new InstantCommand(()-> p_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
                        .alongWith(new InstantCommand(()-> GlobalVariables.m_intakeOn = false))
//                        .alongWith(new InstantCommand(p_turntable::endIntakeMode))
                        .alongWith(new CMD_IntakeInterrupt(p_turntable)),

                new InstantCommand(()-> p_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
                        .alongWith(new InstantCommand(()-> GlobalVariables.m_intakeOn = true))
//                        .alongWith(new InstantCommand(p_turntable::setTurntableIntakeMode))
                        .alongWith(new CMD_Intake(p_turntable)),

                () -> GlobalVariables.m_intakeOn
            )
        );
    }
}
