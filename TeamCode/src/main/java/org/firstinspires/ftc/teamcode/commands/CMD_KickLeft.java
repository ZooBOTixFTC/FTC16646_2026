package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Kicker;


public class CMD_KickLeft extends CommandBase {
    private final ElapsedTime timer = new ElapsedTime();
    private final SUB_Kicker m_kicker;

    private boolean kicked;
    private boolean isFinished;

    public CMD_KickLeft(SUB_Kicker p_kicker) {
        m_kicker = p_kicker;
        addRequirements(m_kicker);
    }

    @Override
    public void initialize() {
        timer.reset();
        kicked = false;
        isFinished = false;

        m_kicker.kickLeft();
    }

    @Override
    public void execute(){
        if(!kicked && timer.seconds() > Constants.KickConstants.kKickDur){
            m_kicker.homeLeft();
            kicked = true;

            timer.reset();
        }

        if(kicked && timer.seconds() > Constants.KickConstants.kKickDur){
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        //finish after reaching target pos and having kicked
        return isFinished;
    }
}
