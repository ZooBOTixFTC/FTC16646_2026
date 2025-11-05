package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Intake;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Lift;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Vision;

public class CMD_Shoot extends SequentialCommandGroup {
    public CMD_Shoot(MecanumDriveSubsystem p_drive, SUB_Shooter p_shooter,
                     SUB_Lift p_lift, SUB_Intake p_intake, SUB_Vision p_vision, GamepadEx p_driverOp){
        addRequirements(p_drive, p_shooter);

        addCommands(
//            new CMD_AlignTarget(p_drive, p_vision, p_driverOp)
            new CMD_AdjustTargetVel(p_shooter)
            ,new CMD_GetShooterAtVelocity(p_shooter)
            ,new InstantCommand(()-> p_lift.setTargetPos(Constants.LiftConstants.kKick))
            ,new WaitCommand(250)
            ,new InstantCommand(()-> p_lift.setTargetPos(Constants.LiftConstants.kHome))
            ,new WaitCommand(500)
            ,new CMD_GetShooterAtVelocity(p_shooter)
            ,new InstantCommand(()-> p_intake.setMotorPower(Constants.IntakeConstants.kIntakeOn))
            ,new InstantCommand(()-> p_lift.setTargetPos(Constants.LiftConstants.kKick))
            ,new WaitCommand(750)
            ,new InstantCommand(()-> p_lift.setTargetPos(Constants.LiftConstants.kHome))
            ,new WaitCommand(500)
        );
    }
}
