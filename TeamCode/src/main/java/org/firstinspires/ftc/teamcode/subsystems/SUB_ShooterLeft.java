package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

@Config
public class SUB_ShooterLeft extends SubsystemBase {
    private final OpMode m_opMode;
    private final DcMotorEx m_shooter;
    private final FtcDashboard m_dashboard;
    private final VoltageSensor m_voltageSensor;
    private final ElapsedTime m_time = new ElapsedTime();

    private double targetVel, lastTime, smoothedVel, lastError, volts, deltaTime = 0;
    private int lastPos = 0;

    public static double kP, kD, kS, kV, vel = 0;

    private boolean unjamming;

    public SUB_ShooterLeft(OpMode p_opMode) {
        m_opMode = p_opMode;

        m_time.reset();

        m_dashboard = FtcDashboard.getInstance();
        m_dashboard.setTelemetryTransmissionInterval(20);

        m_voltageSensor = m_opMode.hardwareMap.voltageSensor.iterator().next();

        m_shooter = m_opMode.hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");

        m_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_shooter.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void unjam(){
        targetVel = 0;
        m_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_shooter.setPower(-1);
        unjamming = true;
    }

    public void stop(){
        m_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_shooter.setPower(-1);
        unjamming = false;
    }

    public double getVel() {
        double currentTime = m_time.seconds();
        deltaTime = (currentTime - lastTime);

        if(deltaTime > .1) {
            int currentPos = m_shooter.getCurrentPosition();
            int deltaTicks = currentPos - lastPos;

            double ticksPerRev = 28.0; // 1:1 GoBilda motor
            double revs = deltaTicks / ticksPerRev;
            double rawVelocity = (revs * 60) / deltaTime;

            smoothedVel = Constants.ShooterConstants.kAlpha * rawVelocity + (1 - Constants.ShooterConstants.kAlpha) * smoothedVel;

            lastPos = currentPos;
            lastTime = currentTime;
        }
//        return Math.round(smoothedVel);
        return smoothedVel;
    }

    public void setGoal(double targetVel){
        this.targetVel = targetVel;
    }

    public void setVel(){
        double error = (Constants.ShooterConstants.kTuningMode ? vel : targetVel) - getVel();
        double errorRate = (error - lastError) / deltaTime;

        lastError = error;

        if(Constants.ShooterConstants.kTuningMode){
            volts =
                kS +
                kV * vel +
                kP * error +
                kD * errorRate;
        }else {
            volts =
                Constants.ShooterConstants.kSRight +
                Constants.ShooterConstants.kVRight * targetVel +
                Constants.ShooterConstants.kPRight * error +
                Constants.ShooterConstants.kDRight * errorRate;
        }

        double power;

        power = volts * (12 / m_voltageSensor.getVoltage());
        power = MathUtils.clamp(power, 0, 1);

        if (!unjamming) m_shooter.setPower(power);
        if(targetVel == 0 && vel == 0 && !unjamming) m_shooter.setPower(0);
    }

    public double getTargetVel(){
        return targetVel;
    }

    @Override
    public void periodic() {
        setVel();

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Left Vel", getVel());
        packet.put("Left Target Vel", Constants.ShooterConstants.kTuningMode ? vel : targetVel);
        packet.put("Left Volts", volts * m_voltageSensor.getVoltage());

        m_dashboard.sendTelemetryPacket(packet);
    }
}