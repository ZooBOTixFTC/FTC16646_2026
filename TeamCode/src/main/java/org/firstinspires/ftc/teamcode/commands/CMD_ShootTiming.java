package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Intake;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Lift;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShootTiming extends SequentialCommandGroup {
    public CMD_ShootTiming(SUB_Shooter p_shooter, SUB_Lift p_lift, SUB_Intake p_intake){
        addRequirements(p_shooter);

        addCommands(
            new CMD_AdjustTargetVel(p_shooter)
            ,new CMD_GetShooterAtVelocity(p_shooter)
            ,new InstantCommand(()-> p_lift.setTargetPos(Constants.LiftConstants.kKick))
            ,new WaitCommand(250)
            ,new InstantCommand(()-> p_shooter.setShootingVelocity(0))
            ,new InstantCommand(()-> p_shooter.runPower(1))
            ,new InstantCommand(()-> p_lift.setTargetPos(Constants.LiftConstants.kLevel))
            ,new WaitCommand(250)
            ,new InstantCommand(()-> p_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
            ,new WaitCommand(750)
            ,new InstantCommand(()-> p_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff))
            ,new InstantCommand(()-> p_shooter.runPower(0))
        );
    }
}
