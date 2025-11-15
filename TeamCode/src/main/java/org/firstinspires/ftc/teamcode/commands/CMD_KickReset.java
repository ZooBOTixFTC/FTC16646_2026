package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

public class CMD_KickReset extends CommandBase {
    SUB_Lift m_lift;
    boolean m_isFinished = false;
    boolean m_isMoving = true;
    int m_previousEncoderValue;

    ElapsedTime m_runTime = new ElapsedTime();

    public CMD_KickReset(SUB_Lift p_lift) {
        m_lift = p_lift;
        addRequirements(p_lift);
    }

    @Override
    public void initialize() {
        m_isFinished = false;
        m_previousEncoderValue = m_lift.getTicks() + 10;
        m_lift.startReset();
        m_isMoving = true;
    }

    @Override
    public void execute(){
        if (m_isMoving) {
            if (m_previousEncoderValue <= m_lift.getTicks()) {
                m_isMoving = false;
                m_runTime.reset();
            }
            m_previousEncoderValue = m_lift.getTicks();
        } else {
            m_lift.reset();
            // wait for the elevator is stabilize before finish
            if (m_runTime.milliseconds() > 100) m_isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }

    @Override
    public void end(boolean interrupted){
        m_lift.reset();
    }
}