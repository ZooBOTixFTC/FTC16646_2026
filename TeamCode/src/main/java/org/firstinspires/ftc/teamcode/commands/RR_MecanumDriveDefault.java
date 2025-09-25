package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.subsystems.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

/**
 * A command to drive the robot with joystick input.
 */
public class RR_MecanumDriveDefault extends CommandBase {

    MecanumDriveSubsystem m_drivetrain;
    private final GamepadEx m_driverOP;
    private final GamepadEx m_toolOp;
    private final double m_driverOffsetAngle;
    private final double m_deadband;
    GlobalVariables m_variables;
    public RR_MecanumDriveDefault(MecanumDriveSubsystem p_drive, GamepadEx driverOp, GamepadEx p_toolOp,
                                  double driverOffsetAngle, double joystickMin, GlobalVariables p_variables) {
        m_drivetrain = p_drive;
        m_driverOP = driverOp; // gamepad of driver
        m_toolOp = p_toolOp;
        m_driverOffsetAngle = driverOffsetAngle;
        m_deadband = joystickMin;
        m_variables = p_variables;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        double leftY = -m_driverOP.getLeftY(); // speed
        double leftX = m_driverOP.getLeftX(); // strafe
        double rightX = m_driverOP.getRightX(); // turn
        leftY = valueSquared(handleDeadband(leftY, m_deadband));
        leftX = valueSquared(handleDeadband(leftX, m_deadband));
        rightX = valueCubed(handleDeadband(rightX, m_deadband));

        double speed = leftY, turn = rightX, strafe = leftX;

        final double slowMax = 0.50;
        double slowMo = m_toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        if (slowMo > slowMax) {
            speed *= slowMax / slowMo;
            strafe *= slowMax / slowMo;
            turn *= slowMax / slowMo;
        }

        m_drivetrain.drive(speed, strafe, turn, m_driverOffsetAngle);
    }

    private double handleDeadband(double value, double deadband) {
        double newValue;

        if (Math.abs(value) <= Math.abs(deadband)) {
            // in the deadband so return zero
            newValue = 0;
        } else {
            // scale and translate the value [deadband] .. [max]
            newValue = ( Math.abs(value) - deadband )/ ( 1 - deadband);
            // apply the original sign
            newValue = newValue * (Math.abs(value) / value);
        }

        return newValue;
    }

    private double valueSquared(double value) {
        return (value * Math.abs(value));
    }

    private double valueCubed(double value) {
        return (value * value * value);
    }

}


