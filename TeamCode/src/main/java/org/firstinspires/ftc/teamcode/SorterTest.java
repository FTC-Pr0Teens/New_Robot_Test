package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Sorter;

@TeleOp(name="Sorter Servo Test", group="Test")
public class SorterTest extends OpMode {
    
    private Sorter sorter;

    @Override
    public void init() {
        sorter = new Sorter(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Button B -> Slot 1 (Index 0)
        if (gamepad1.b) {
            sorter.moveToSlot(0);
            telemetry.addData("Last Command", "Move to Slot 1 (B)");
        }

        // Button A -> Slot 2 (Index 1)
        if (gamepad1.a) {
            sorter.moveToSlot(1);
            telemetry.addData("Last Command", "Move to Slot 2 (A)");
        }

        // Button X -> Slot 3 (Index 2)
        if (gamepad1.x) {
            sorter.moveToSlot(2);
            telemetry.addData("Last Command", "Move to Slot 3 (X)");
        }

        telemetry.addData("Controls", "B: Slot 1, A: Slot 2, X: Slot 3");
        telemetry.update();
    }
}
