package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

/**
 * Utility OpMode to set all servos to their 0.0 position.
 * Useful for physically mounting servo horns.
 */
@TeleOp(name = "Servo Resetter", group = "Test")
public class ServoResetter extends OpMode {
    Hardware hw;

    @Override
    public void init() {
        hw = Hardware.getInstance(hardwareMap);
        telemetry.addData("Status", "Initialized. Press Play to zero all servos.");
    }

    @Override
    public void loop() {
        // Force all servos to 0.0
        hw.sorter1.setPosition(0);
        hw.sorter2.setPosition(0);
        hw.flipper.setPosition(0);
        hw.hood.setPosition(0);

        telemetry.addLine("All servos commanded to 0.0");
        telemetry.addData("Sorter 1", hw.sorter1.getPosition());
        telemetry.addData("Sorter 2", hw.sorter2.getPosition());
        telemetry.addData("Flipper", hw.flipper.getPosition());
        telemetry.addData("Hood", hw.hood.getPosition());
        telemetry.update();
    }
}
