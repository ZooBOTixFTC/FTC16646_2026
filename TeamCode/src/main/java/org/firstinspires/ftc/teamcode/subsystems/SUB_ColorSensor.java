package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

public class SUB_ColorSensor extends SubsystemBase {
    private final NormalizedColorSensor m_colorSensorLeft;
    private final NormalizedColorSensor m_colorSensorRight;
    private final Servo m_LEDRight;
    private final Servo m_LEDLeft;
    private final OpMode m_opMode;
    public SUB_ColorSensor(OpMode p_opMode) {
        m_opMode = p_opMode;
        m_colorSensorLeft = m_opMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensorLeft");
        m_colorSensorRight = m_opMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensorRight");
        m_LEDLeft = m_opMode.hardwareMap.get(Servo.class, "LEDLeft");
        m_LEDRight = m_opMode.hardwareMap.get(Servo.class, "LEDRight");
    }

    // Get the raw color data
    public NormalizedRGBA getColorLeft() {
        return m_colorSensorLeft.getNormalizedColors();
    }

    public NormalizedRGBA getColorRight() {
        return m_colorSensorRight.getNormalizedColors();
    }

    // Detect between green and purple
    NormalizedRGBA colorsLeft;
    NormalizedRGBA colorsRight;
    public String detectColorLeft() {
        colorsLeft = getColorLeft();

        float red = colorsLeft.red;
        float green = colorsLeft.green;
        float blue = colorsLeft.blue;

        // Simple decision logic
        if (green > (red + blue) * 0.75) {
            return "Green";
        } else if ((red + blue) > (green * 1.5)) {
            return "Purple";
        } else {
            return "Unknown";
        }
    }

    public String detectColorRight() {
        colorsLeft = getColorRight();

        float red = colorsRight.red;
        float green = colorsRight.green;
        float blue = colorsRight.blue;

        // Simple decision logic
        if (green > (red + blue) * 0.75) {
            return "Green";
        } else if ((red + blue) > (green * 1.5)) {
            return "Purple";
        } else {
            return "Unknown";
        }
    }

    public void setLEDLeftColor(double color) {
        m_LEDLeft.setPosition(color);
    }

    public void setLEDRightColor(double color) {
        m_LEDRight.setPosition(color);
    }

    // Telemetry for debugging
    @Override
    public void periodic() {
        m_opMode.telemetry.addData("Red", colorsLeft.red);
        m_opMode.telemetry.addData("Green", colorsLeft.green);
        m_opMode.telemetry.addData("Blue", colorsLeft.blue);
        m_opMode.telemetry.addData("Detected Left", detectColorLeft());
        m_opMode.telemetry.addData("Red", colorsRight.red);
        m_opMode.telemetry.addData("Green", colorsRight.green);
        m_opMode.telemetry.addData("Blue", colorsRight.blue);
        m_opMode.telemetry.addData("Detected Right", detectColorRight());
    }
}
