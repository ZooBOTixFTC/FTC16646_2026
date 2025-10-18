package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class SUB_turnTable extends SubsystemBase {

    private final DcMotorEx m_turntableMotor;
    private final OpMode m_opMode;

    public SUB_turnTable(OpMode p_opMode){
        m_opMode = p_opMode;
        m_turntableMotor = m_opMode.hardwareMap.get(DcMotorEx.class, "turntableMotor");
        m_turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public double getPos() {
        double deg = (m_turntableMotor.getCurrentPosition() / Constants.ShooterConstants.kTicksPerRev) * 360.0;
        return deg % 360;
    }
    public void changeTurntableTargetPos(double targetDeg) {
        int targetTicks = (int) ((targetDeg / 360) * Constants.ShooterConstants.kTicksPerRev);
        targetTicks += m_turntableMotor.getCurrentPosition();
        m_turntableMotor.setTargetPosition(targetTicks);
        m_turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_turntableMotor.setPower(1);
    }
    public void setTurntableIntakeMode() {
        m_turntableMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_turntableMotor.setPower(.25);
    }
    public void endIntakeMode() {
        m_turntableMotor.setPower(0);
        ArrayList<Double> distances = new ArrayList<>(Arrays.asList((0-getPos()),(120-getPos()),(240-getPos()),(360-getPos())));
        int minIndex = distances.indexOf(Collections.min(distances));
        changeTurntableTargetPos(distances.get(minIndex));
    }

    @Override
    public void periodic(){
        m_opMode.telemetry.addData("position",m_turntableMotor.getCurrentPosition());
    }
}
