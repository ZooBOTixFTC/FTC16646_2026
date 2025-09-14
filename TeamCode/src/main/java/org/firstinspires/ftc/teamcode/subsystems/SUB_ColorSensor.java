package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class SUB_ColorSensor extends SubsystemBase {

    private final NormalizedColorSensor colorSensor;
    private final OpMode m_OpMode;
    private double hue;
    private NormalizedRGBA colors;
    public SUB_ColorSensor(OpMode p_OpMode) {

        m_OpMode = p_OpMode;
        colorSensor = m_OpMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    public void hsvColorDetection() {
        float hsv[] = new float[3];
        colors = colorSensor.getNormalizedColors();

        Color.RGBToHSV(
                (int)(colors.red * 255),
        (int)(colors.green * 255),
        (int)(colors.blue * 255),
                hsv
        );
        hue = hsv[0];
    }

    public String detectedColor() {

        if (hue >= 145 && hue <= 159) {
            return "Green";
        } else if (hue >= 170 && hue <= 237) {
            return "Purple";
        }
        else return "Unknown";
    }

    @Override
    public void periodic() {
        hsvColorDetection();
        m_OpMode.telemetry.addData("detected color", detectedColor());
        m_OpMode.telemetry.addData("Red", colors.red);
        m_OpMode.telemetry.addData("Green", colors.green);
        m_OpMode.telemetry.addData("Blue", colors.blue);
        m_OpMode.telemetry.addData("Hue", hue);

    }
}
