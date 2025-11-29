package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Intake;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Kicker;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Shooter;

public class CMD_ShooterDefault extends CommandBase {
    private final SUB_Shooter m_shooter;
    private final SUB_Intake m_intake;
    private final SUB_Kicker m_kicker;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean triggered;
    public CMD_ShooterDefault(SUB_Shooter p_shooter, SUB_Intake p_intake, SUB_Kicker p_kicker){
        m_shooter = p_shooter;
        m_intake = p_intake;
        m_kicker = p_kicker;

        addRequirements(m_shooter);
    }

    @Override
    public void initialize(){
        timer.reset();
        triggered = false;
    }

    @Override
    public void execute(){
        if (timer.milliseconds() > 250 && !triggered) {
            m_shooter.setShootingVelocity(0.0);

            m_intake.setMotorPower(Constants.IntakeConstants.kIntakeOff);

            m_kicker.home();

            triggered = true;
        }
    }
}
