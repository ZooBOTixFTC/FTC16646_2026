package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;
import java.util.List;
import java.util.Timer;

public class CMD_ReadPattern extends CommandBase {

    private List<AprilTagDetection> m_detections;
    private final SUB_Vision m_vision;
    private final ElapsedTime m_elapsedTime = new ElapsedTime();

    public CMD_ReadPattern(SUB_Vision p_vision) {
        m_vision = p_vision;
    }

    @Override
    public void initialize() {
        m_elapsedTime.reset();
    }

    @Override
    public void execute() {
        m_detections = m_vision.getDetections();
        try {
            for (AprilTagDetection detection : m_detections) {
                switch (detection.id) {
                    case 21:
                        GlobalVariables.m_patternType = GlobalVariables.patternTypes.GPP;
                        break;
                    case 22:
                        GlobalVariables.m_patternType = GlobalVariables.patternTypes.PGP;
                        break;
                    case 23:
                        GlobalVariables.m_patternType = GlobalVariables.patternTypes.PPG;
                    break;
                }
            }
        }catch (Exception e){
            System.out.println(Arrays.toString(e.getStackTrace()));
        }
    }

    @Override
    public boolean isFinished() {
        return m_elapsedTime.milliseconds()>500;
    }
}
