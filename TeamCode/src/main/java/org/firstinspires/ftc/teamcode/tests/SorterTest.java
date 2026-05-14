package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Sorter;

@TeleOp(name = "Sorter Tuning", group = "Tuning")
public class SorterTest extends OpMode {

    Hardware hw;
    Sorter sorter;

    private double pos1 = 0.19;  // Slot 0
    private double pos2 = 0.54;   // Slot 1
    private double pos3 = 0.90;  // Slot 2

    private int selectedSlot = 0;

    private static final double STEP = 0.01;

    private Servo.Direction dir1 = Servo.Direction.FORWARD;

    // Button edge detection
    private boolean lastDpadUp    = false;
    private boolean lastDpadDown  = false;
    private boolean lastDpadRight = false;
    private boolean lastLB        = false;
    private boolean lastRB        = false;
    private boolean lastA         = false; // toggle sorter1 direction
    private boolean lastX         = false; // jump to slot 1
    private boolean lastY         = false; // jump to slot 2
    private boolean lastB         = false; // jump to slot 3

    @Override
    public void init() {
        hw     = Hardware.getInstance(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry);

        hw.sorter1.setDirection(dir1);
        applySlot(selectedSlot);

        telemetry.addLine("Sorter Tuning ready.");
        telemetry.update();
    }

    @Override
    public void loop() {

        // --- Slot selection: dpad up / down ---
        boolean dpadUp   = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        if (dpadUp && !lastDpadUp) {
            selectedSlot = (selectedSlot + 1) % 3;
            applySlot(selectedSlot);
        }
        if (dpadDown && !lastDpadDown) {
            selectedSlot = (selectedSlot + 2) % 3;
            applySlot(selectedSlot);
        }
        lastDpadUp   = dpadUp;
        lastDpadDown = dpadDown;

        // --- Direct slot jump: X / Y / B ---
        boolean xBtn = gamepad1.x;
        boolean yBtn = gamepad1.y;
        boolean bBtn = gamepad1.b;

        if (xBtn && !lastX) { selectedSlot = 0; applySlot(selectedSlot); }
        if (yBtn && !lastY) { selectedSlot = 1; applySlot(selectedSlot); }
        if (bBtn && !lastB) { selectedSlot = 2; applySlot(selectedSlot); }
        lastX = xBtn;
        lastY = yBtn;
        lastB = bBtn;

        // --- Fine-tune selected slot position: LB (-) / RB (+) ---
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        if (lb && !lastLB) adjustSlot(selectedSlot, -STEP);
        if (rb && !lastRB) adjustSlot(selectedSlot, +STEP);
        lastLB = lb;
        lastRB = rb;

        // --- Mirror selected slot position: dpad right ---
        boolean dpadRight = gamepad1.dpad_right;
        if (dpadRight && !lastDpadRight) mirrorSlot(selectedSlot);
        lastDpadRight = dpadRight;

        // --- Toggle sorter1 direction: A ---
        boolean aBtn = gamepad1.a;
        if (aBtn && !lastA) {
            dir1 = (dir1 == Servo.Direction.FORWARD)
                    ? Servo.Direction.REVERSE
                    : Servo.Direction.FORWARD;
            hw.sorter1.setDirection(dir1);
            applySlot(selectedSlot);
        }
        lastA = aBtn;

        // --- Telemetry ---
        telemetry.addLine("========= Sorter Tuning =========");
        telemetry.addLine("dpad up/down  → cycle slot");
        telemetry.addLine("X / Y / B     → jump to slot 1 / 2 / 3");
        telemetry.addLine("LB / RB       → adjust position  (-/+0.01)");
        telemetry.addLine("dpad right    → mirror position  (1.0 - pos)");
        telemetry.addLine("A             → flip sorter1 direction");
        telemetry.addLine("");
        telemetry.addData("Selected Slot", selectedSlot + 1);
        telemetry.addData("Slot 1 (pos1)", "%.3f %s", pos1, selectedSlot == 0 ? " <--" : "");
        telemetry.addData("Slot 2 (pos2)", "%.3f %s", pos2, selectedSlot == 1 ? " <--" : "");
        telemetry.addData("Slot 3 (pos3)", "%.3f %s", pos3, selectedSlot == 2 ? " <--" : "");
        telemetry.addLine("");
        telemetry.addData("sorter1 direction", dir1);
        telemetry.addData("sorter1 pos (reported)", "%.3f", hw.sorter1.getPosition());
        telemetry.update();
    }

    private void applySlot(int slot) {
        hw.sorter1.setPosition(getPosForSlot(slot));
    }

    private void adjustSlot(int slot, double delta) {
        switch (slot) {
            case 0: pos1 = clamp(pos1 + delta); break;
            case 1: pos2 = clamp(pos2 + delta); break;
            case 2: pos3 = clamp(pos3 + delta); break;
        }
        applySlot(slot);
    }

    private void mirrorSlot(int slot) {
        switch (slot) {
            case 0: pos1 = clamp(1.0 - pos1); break;
            case 1: pos2 = clamp(1.0 - pos2); break;
            case 2: pos3 = clamp(1.0 - pos3); break;
        }
        applySlot(slot);
    }

    private double getPosForSlot(int slot) {
        switch (slot) {
            case 0:  return pos1;
            case 1:  return pos2;
            case 2:  return pos3;
            default: return 0.5;
        }
    }

    private double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}