package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

public class SUB_turntable extends SubsystemBase {

    private final DcMotorEx m_turntableMotor;
    private final OpMode m_opMode;

    public SUB_turntable(OpMode p_opMode){
        m_opMode = p_opMode;
        m_turntableMotor = m_opMode.hardwareMap.get(DcMotorEx.class, "turntableMotor");

        m_turntableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_turntableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_turntableMotor.setTargetPosition(0);
        m_turntableMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        m_turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public double getPos() {
        double deg = (m_turntableMotor.getCurrentPosition() / ShooterConstants.kTicksPerRev) * 360.0;
        return deg % 360;
    }

    public void rotateLeft(){
        setPos(getPos() - 120);
    }

    public void rotateRight(){
        setPos(getPos() + 120);
    }

    public void setPos(double targetDeg) {
        int currentTicks = m_turntableMotor.getCurrentPosition();
        double currentRev = (double) currentTicks / ShooterConstants.kTicksPerRev;

        // Convert target angle to ticks within one revolution
        int targetTicksInRev = (int) ((targetDeg / 360.0) * ShooterConstants.kTicksPerRev);

        // Find the closest tick position to currentTicks that matches targetDeg
        int baseRev = (int) Math.round(currentRev); // Nearest full revolution
        int candidateTicks = (int) (baseRev * ShooterConstants.kTicksPerRev + targetTicksInRev);

        // If candidate is too far, adjust by ±1 rev to get closest match
        int alt1 = (int) (candidateTicks + ShooterConstants.kTicksPerRev);
        int alt2 = (int) (candidateTicks - ShooterConstants.kTicksPerRev);

        int bestTarget = candidateTicks;
        if (Math.abs(alt1 - currentTicks) < Math.abs(bestTarget - currentTicks)) {
            bestTarget = alt1;
        }
        if (Math.abs(alt2 - currentTicks) < Math.abs(bestTarget - currentTicks)) {
            bestTarget = alt2;
        }

        m_turntableMotor.setTargetPosition(bestTarget);
        m_turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_turntableMotor.setPower(1);
    }

    public void endIntakeMode() {
        double[] angToCheck = {0, 120, 240};
        double current = getPos();
        double closestAng = angToCheck[0];

        for (int i = 1; i < angToCheck.length; i++) {
            if (Math.abs(angToCheck[i] - current) < Math.abs(closestAng - current)) {
                closestAng = angToCheck[i];
            }
        }

        setPos(closestAng);
    }

    public void setTurntableIntakeMode() {
        m_turntableMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_turntableMotor.setPower(.2);
    }

    @Override
    public void periodic(){
        m_opMode.telemetry.addData("turntable pos", getPos());
        m_opMode.telemetry.addData("turntable ticks", m_turntableMotor.getCurrentPosition());

    }
}
