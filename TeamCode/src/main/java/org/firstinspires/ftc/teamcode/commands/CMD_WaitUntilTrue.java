package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.function.Supplier;

public class CMD_WaitUntilTrue extends CommandBase {

    GamepadKeys.Button m_button;
    GamepadEx m_gamepad;
    public CMD_WaitUntilTrue(GamepadEx p_gamepad, GamepadKeys.Button p_button){
        m_gamepad = p_gamepad;
        m_button = p_button;
    }

    @Override
    public boolean isFinished(){
        return !m_gamepad.getButton(m_button);
    }
}
