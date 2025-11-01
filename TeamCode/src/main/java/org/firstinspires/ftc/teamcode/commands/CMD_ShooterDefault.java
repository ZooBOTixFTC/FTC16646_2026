package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Intake;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ShooterLeft;
import org.firstinspires.ftc.teamcode.subsystems.SUB_ShooterRight;

public class CMD_ShooterDefault extends CommandBase {
    private final SUB_ShooterLeft m_shooterLeft;
    private final SUB_ShooterRight m_shooterRight;
    private final SUB_Intake m_intake;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean triggered;
    public CMD_ShooterDefault(SUB_ShooterLeft p_shooterLeft, SUB_ShooterRight p_shooterRight, SUB_Intake p_intake){
        m_shooterLeft = p_shooterLeft;
        m_shooterRight = p_shooterRight;
        m_intake = p_intake;

        addRequirements(m_shooterLeft, m_shooterRight);
    }

    @Override
    public void initialize(){
        timer.reset();
        triggered = false;
    }

    @Override
    public void execute(){
        if (timer.milliseconds() > 500 && !triggered) {
            m_shooterLeft.setGoal(0);
            m_shooterRight.setGoal(0);
            m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff);
            triggered = true;
        }
    }
}
