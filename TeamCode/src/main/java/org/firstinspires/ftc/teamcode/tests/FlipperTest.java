package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp(name="Flipper Servo Test", group="Test")
public class FlipperTest extends OpMode {
    
    private Hardware hw;
    private double currentPos = 0.5;
    
    // Toggle trackers
    private boolean lastRB = false;
    private boolean lastLB = false;

    @Override
    public void init() {
        // ALWAYS re-initialize to ensure fresh settings
        hw = new Hardware(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Toggle Direction (A Button)

        // Step by 0.05 (Bumpers)
        if (gamepad1.right_bumper && !lastRB) {
            currentPos += 0.05;
        }
        lastRB = gamepad1.right_bumper;

        if (gamepad1.left_bumper && !lastLB) {
            currentPos -= 0.05;
        }
        lastLB = gamepad1.left_bumper;

        // Manual sweep (Left Stick)
        if (Math.abs(gamepad1.left_stick_y) > 0.05) {
            currentPos = (1.0 - ((gamepad1.left_stick_y + 1.0) / 2.0));
        }

        currentPos = Math.max(0, Math.min(1, currentPos));
        hw.flipper.setPosition(currentPos);

        telemetry.addData("Commanded Pos", "%.2f", currentPos);
        telemetry.addData("Direction", hw.flipper.getDirection());
        telemetry.addLine();
        telemetry.addData("Stick Y", "Free Sweep (0 to 1)");
        telemetry.addData("Bumpers", "+/- 0.05 steps");
        telemetry.addData("A Button", "Toggle FORWARD/REVERSE");
        telemetry.update();
    }
}
