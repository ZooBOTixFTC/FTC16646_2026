package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SUB_Drivetrain;

public class RR_TrajectoryFollowerCommand extends CommandBase {
    private final SUB_Drivetrain m_drivetrain;
    private final Trajectory m_trajectory;
    private Action m_trajectoryAction;
    private boolean isFinished;

    public RR_TrajectoryFollowerCommand(SUB_Drivetrain p_drivetrain, Trajectory p_trajectory) {
        m_drivetrain = p_drivetrain;
        m_trajectory = p_trajectory;
    }

    @Override
    public void initialize() {
        isFinished = false;
        // Initialize the trajectory action (ensure the action is compatible with Trajectory)
        m_trajectoryAction = m_drivetrain.runTrajectory(m_trajectory);
    }

    @Override
    public void execute() {
        // Run the trajectory action and use telemetry for debugging
        TelemetryPacket packet = new TelemetryPacket();
        boolean isRunning = m_trajectoryAction.run(packet);
        isFinished = !isRunning;
    }

    @Override
    public boolean isFinished() {
        // Check if the trajectory action has completed
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command finishes
        m_drivetrain.drive(0, 0, 0);
    }
}
