package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

@Config
public class SUB_Shooter extends SubsystemBase {
    private final SUB_ShooterLeft m_shooterLeft;
    private final SUB_ShooterRight m_shooterRight;

    public SUB_Shooter(SUB_ShooterLeft p_shooterLeft, SUB_ShooterRight p_shooterRight) {
        m_shooterLeft = p_shooterLeft;
        m_shooterRight = p_shooterRight;
    }

    public void setGoal(double targetVel){
        m_shooterLeft.setGoal(targetVel);
        m_shooterRight.setGoal(targetVel);
    }

    public double getTargetVel(){
        return m_shooterLeft.getTargetVel();
    }

    public double getVel(){
        return m_shooterLeft.getVel();
    }

    public void unjam(){
        m_shooterLeft.unjam();
        m_shooterRight.unjam();
    }

    public void stop(){
        m_shooterLeft.stop();
        m_shooterRight.stop();
    }
}
