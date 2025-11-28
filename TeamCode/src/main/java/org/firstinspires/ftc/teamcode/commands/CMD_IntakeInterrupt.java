package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SUB_Turntable;

public class CMD_IntakeInterrupt extends CommandBase {
    public CMD_IntakeInterrupt(SUB_Turntable p_turntable) {
        addRequirements(p_turntable);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}