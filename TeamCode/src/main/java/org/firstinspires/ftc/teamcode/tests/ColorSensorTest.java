package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp(name = "Color Sensor Test", group = "Test")
public class ColorSensorTest extends OpMode {
    private Hardware hw;

    @Override
    public void init() {
        hw = Hardware.getInstance(hardwareMap);
        // Set gain for better detection (adjust as needed)
        hw.ncs.setGain(2.0f);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Get normalized colors from the sensor
        NormalizedRGBA colors = hw.ncs.getNormalizedColors();

        // Calculate hue using standard FTC utility
        double hue = JavaUtil.colorToHue(colors.toColor());

        // Display data
        telemetry.addLine("Hold a ball in front of the sensor");
        telemetry.addData("Hue", "%.1f", hue);
        telemetry.addData("Alpha (Brightness)", "%.3f", colors.alpha);
        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);

        // Help user identify colors based on typical ranges
        String detected = "NONE";
        if (colors.alpha > 0.1) { // Only classify if something is actually there
            if (hue > 200 && hue < 375) detected = "PURPLE";
            else if (hue > 65 && hue < 175) detected = "GREEN";
        }
        telemetry.addData("Classification", detected);

        telemetry.update();
    }
}
