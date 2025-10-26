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

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

@Config
public class SUB_Shooter extends SubsystemBase {
    private final OpMode m_opMode;
    private final DcMotorEx m_shooterMotor;
    private final VoltageSensor m_voltageSensor;
    private final FtcDashboard m_dashboard;
    private double m_targetVel;
    private double lastVelError = 0;

    public static double P, D, V, S, Vel;

    private int lastPos = 0;
    private long lastTime = System.nanoTime();
    private double smoothedVelocity = 0;

    public SUB_Shooter(OpMode p_opMode) {
        m_opMode = p_opMode;
        m_shooterMotor = m_opMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");

        m_voltageSensor = m_opMode.hardwareMap.voltageSensor.iterator().next();

        m_dashboard = FtcDashboard.getInstance();
        m_dashboard.setTelemetryTransmissionInterval(20);

        m_shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        m_shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setTargetVel(double velocity) {
        m_targetVel = velocity;
    }


    public double getVelocity() {
        long currentTime = System.nanoTime();
        double deltaTimeSec = (currentTime - lastTime) / 1e9; // convert nanoseconds to milliseconds

        if (deltaTimeSec >= 0.02) { // 20ms loop
            int currentPos = m_shooterMotor.getCurrentPosition();
            int deltaTicks = currentPos - lastPos;

            double ticksPerRev = 28.0; // 1:1 GoBilda motor
            double revs = deltaTicks / ticksPerRev;
            double rawVelocity = (revs * 360.0) / deltaTimeSec;

            smoothedVelocity = ShooterConstants.kAlpha * rawVelocity + (1 - ShooterConstants.kAlpha) * smoothedVelocity;

            lastPos = currentPos;
            lastTime = currentTime;
        }

        return Math.round(smoothedVelocity);
    }

    public double getTargetVel(){
        return m_targetVel;
    }


    @Override
    public void periodic() {
        m_opMode.telemetry.addData("Shooter Vel", getVelocity());
        m_opMode.telemetry.addData("Shooter Target Vel", getTargetVel());

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("left vel", getVelocity());
        packet.put("left target vel", getTargetVel());


        m_dashboard.sendTelemetryPacket(packet);

        double Volts;

        double VelError = getTargetVel() - getVelocity();

        double ErrorRate = (VelError - lastVelError) / .02;

        lastVelError = VelError;

        if(ShooterConstants.kTuningMode){
            setTargetVel(Vel);

            Volts =
                    S +
                            V * getTargetVel() +
                            P * (getTargetVel() - getVelocity()) +
                            D * ErrorRate;
        }else{
            Volts =
                    ShooterConstants.kS +
                            ShooterConstants.kV * getTargetVel() +
                            ShooterConstants.kP * (getTargetVel() - getVelocity()) +
                            ShooterConstants.kD * ErrorRate;
        }

        double CompensatedPower = MathUtils.clamp(Volts * (12 / m_voltageSensor.getVoltage()), -1, 1);

        if(m_targetVel == 0) CompensatedPower = 0;

        m_shooterMotor.setPower(CompensatedPower);
    }
}