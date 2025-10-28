package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.GlobalVariables;

public class SUB_Turntable extends SubsystemBase {

    private final DcMotorEx m_turntableMotor;
    private final OpMode m_opMode;
    private double m_targetDegs = 0;

    public SUB_Turntable(OpMode p_opMode){
        m_opMode = p_opMode;
        m_turntableMotor = m_opMode.hardwareMap.get(DcMotorEx.class, "turntableMotor");

        m_turntableMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m_turntableMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m_turntableMotor.setTargetPosition(0);
        m_turntableMotor.setDirection(DcMotorEx.Direction.FORWARD);
        m_turntableMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public double getPos() {
        double deg = (m_turntableMotor.getCurrentPosition() / ShooterConstants.kTicksPerRev) * 360.0;
        return deg % 360;
    }

    public void rotateLeft(){
        if (m_targetDegs >= 360) setPos(240);
        else if (m_targetDegs >= 240) setPos(120);
        else if (m_targetDegs >= 120) setPos(0);
        else if (m_targetDegs >= 0) setPos(240);
    }

    public void rotateRight(){
//        if (!GlobalVariables.m_intakeOn) {
            if (m_targetDegs >= 360) setPos(120);
            else if (m_targetDegs >= 240) setPos(360);
            else if (m_targetDegs >= 120) setPos(240);
            else if (m_targetDegs >= 0) setPos(120);
//        }
//        else {
//            if (m_targetDegs >= 360) setPos(60);
//            else if (m_targetDegs >= 240) setPos(300);
//            else if (m_targetDegs >= 120) setPos(180);
//            else if (m_targetDegs >= 0) setPos(60);
//        }
    }

    public void setPos(double targetDeg) {
        m_targetDegs = targetDeg;
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
        m_turntableMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        m_turntableMotor.setPower(0.75);
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
        m_turntableMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m_turntableMotor.setPower(0.3);
    }

    public double getTurntableVelo() {
        return m_turntableMotor.getVelocity();
    }

    @Override
    public void periodic(){
        m_opMode.telemetry.addData("turntable pos", getPos());
        m_opMode.telemetry.addData("turntable ticks", m_turntableMotor.getCurrentPosition());
        m_opMode.telemetry.addData("kicker Homed",GlobalVariables.m_kickerHomed);

    }
}
