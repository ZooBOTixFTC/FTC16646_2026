package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Turntable;

public class CMD_AutoColorSwap extends CommandBase {

    private final SUB_Turntable m_turntable;

    public CMD_AutoColorSwap(SUB_Turntable p_turntable) {
        m_turntable = p_turntable;
    }

    @Override
    public void initialize() {
        switch (GlobalVariables.m_patternType) {
            case GPP:
                break;
            case PGP:
                m_turntable.rotateLeft();
                break;
            case PPG:
                m_turntable.rotateRight();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
