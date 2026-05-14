package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.ShooterSubsystem;

@TeleOp(name = "Shooter Tuning Op", group = "Tuning")
public class ShooterTest extends OpMode {
    private ShooterSubsystem shooter;

    // Configurable runtime test targets
    private double tuneRPM = 2200.0;
    private double testPower = 0.45; // Starts at ~45% raw motor output

    // Debounce state flags for buttons
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    @Override
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);
        telemetry.addData("Status", "Tuning OpMode Initialized");
    }

    @Override
    public void loop() {
        // --- Adjust PID Target RPM (D-Pad Up/Down) ---
        if (gamepad1.dpad_up && !lastDpadUp) tuneRPM += 100.0;
        if (gamepad1.dpad_down && !lastDpadDown) tuneRPM -= 100.0;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // --- Adjust Non-PID Raw Power (D-Pad Left/Right) ---
        if (gamepad1.dpad_right && !lastDpadRight) testPower = Math.min(1.0, testPower + 0.05);
        if (gamepad1.dpad_left && !lastDpadLeft) testPower = Math.max(0.0, testPower - 0.05);
        lastDpadRight = gamepad1.dpad_right;
        lastDpadLeft = gamepad1.dpad_left;

        // --- Mode Execution Handling ---
        if (gamepad1.x) {
            // Test Option 1: Standard Closed-Loop PID
            shooter.setClosedLoop();
            shooter.setTargetRPM(tuneRPM);
            shooter.on();
        } else if (gamepad1.y) {
            // Test Option 2: Raw Test Without PID (Open-Loop)
            shooter.setOpenLoop(testPower);
        } else if (gamepad1.b) {
            // Stop motor safe state
            shooter.off();
        }

        // --- Subsystem Execution Loop ---
        // Your current code handles the conditional paths inside update()
        shooter.update();

        // --- Diagnostic Telemetry Logs ---
        telemetry.addLine("=== SYSTEM MODES ===");
        telemetry.addLine("X: Run PID | Y: Run Raw (No PID) | B: Stop Motor");
        telemetry.addLine("Dpad Up/Dn: Adjust RPM | Dpad L/R: Adjust Power");
        telemetry.addLine("====================");
        telemetry.addData("Control Strategy", shooter.isOpenLoop() ? "OPEN-LOOP (RAW)" : "CLOSED-LOOP (PID)");
        telemetry.addData("Configured Target RPM", tuneRPM);
        telemetry.addData("Configured Target Power", "%.2f", testPower);
        telemetry.update();
    }
}
