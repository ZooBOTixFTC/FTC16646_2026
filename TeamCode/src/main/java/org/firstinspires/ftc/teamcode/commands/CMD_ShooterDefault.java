package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShooterDefault extends CommandBase {
    private final SUB_Shooter m_shooter;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean triggered;
    public CMD_ShooterDefault(SUB_Shooter p_shooter){
        m_shooter = p_shooter;

        addRequirements(m_shooter);
    }

    @Override
    public void initialize(){
        timer.reset();
        triggered = false;
    }

    @Override
    public void execute(){
        if (timer.seconds() > 2 && !triggered) {
            m_shooter.setVelocity(0);
            m_shooter.setFeederPower(ShooterConstants.kFeedOff);
            m_shooter.setKickLeftPos(ShooterConstants.kKickHome);
            m_shooter.setKickRightPos(ShooterConstants.kKickHome);
            triggered = true;
        }
    }
}
