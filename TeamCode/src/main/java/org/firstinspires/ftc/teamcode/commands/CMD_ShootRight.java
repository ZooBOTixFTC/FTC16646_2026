package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShootRight extends CommandBase {
    private final SUB_Shooter m_shooter;
    private final GamepadEx m_gamepad;
    private final RobotContainer m_robot;
    public CMD_ShootRight(SUB_Shooter p_shooter, RobotContainer p_robot, GamepadEx p_gamepad){
        m_shooter = p_shooter;
        m_robot = p_robot;
        m_gamepad = p_gamepad;
        addRequirements(p_shooter);
    }

    @Override
    public void initialize(){
        m_robot.schedule(
            new SequentialCommandGroup(
                new InstantCommand(()-> m_shooter.setVelocity(ShooterConstants.kMaxVelDegPerSec)),
                new CMD_GetShooterAtVelocity(m_shooter),
                new InstantCommand(()-> m_shooter.setFeederPower(ShooterConstants.kFeedPowerOn)),
                new InstantCommand(()-> m_shooter.setKickRightPower(ShooterConstants.kKickOn))
            )
        );
    }

    @Override
    public boolean isFinished(){
        return !m_gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER);
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.setKickRightPower(ShooterConstants.kKickOff);
    }
}
