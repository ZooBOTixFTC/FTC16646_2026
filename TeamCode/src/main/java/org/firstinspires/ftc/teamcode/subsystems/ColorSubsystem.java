package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;

public class ColorSubsystem extends SubsystemBase {
    private NormalizedColorSensor m_colorSensor;
    private OpMode m_opMode;

    public ColorSubsystem(OpMode p_opMode) {
        m_opMode = p_opMode;
        m_colorSensor = m_opMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    // Get the raw color data
    public NormalizedRGBA getColor() {
        return m_colorSensor.getNormalizedColors();
    }

    // Detect between green and purple
    public String detectColor() {
        NormalizedRGBA colors = getColor();

        float red = colors.red;
        float green = colors.green;
        float blue = colors.blue;

        // Simple decision logic
        if (green > (red + blue) * 0.75) {
            return "Green";
        } else if ((red + blue) > (green * 1.5)) {
            return "Purple";
        } else {
            return "Unknown";
        }
    }

    // Telemetry for debugging
    @Override
    public void periodic() {
        NormalizedRGBA colors = getColor();
        m_opMode.telemetry.addData("Red", colors.red);
        m_opMode.telemetry.addData("Green", colors.green);
        m_opMode.telemetry.addData("Blue", colors.blue);
        m_opMode.telemetry.addData("Detected", detectColor());
    }
}
