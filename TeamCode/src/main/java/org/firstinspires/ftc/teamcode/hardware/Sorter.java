package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class Sorter {

    public enum BallColor { PURPLE, GREEN, NONE }

    private static final double POS_1 = 0.19;
    private static final double POS_2 = 0.55;
    private static final double POS_3 = 0.91;

    // Small trim to compensate for mechanical asymmetry between the two servos.
    // Increase if sorter2 still fights; decrease (or negate) if it overshoots.
    private static final double SERVO2_OFFSET = 0.05;

    private Hardware hw;
    private Telemetry telemetry;
    private int currentSlot = 0;
    private boolean ballDetected = false;
    private int lastCommandedSlot = -1;

    private BallColor[] recordedColors = { BallColor.NONE, BallColor.NONE, BallColor.NONE };
    private boolean[] slotUsed = { false, false, false };

    private ElapsedTime elapsedTime;

    public Sorter(HardwareMap hwmap, Telemetry telemetry) {
        hw = Hardware.getInstance(hwmap);
        this.telemetry = telemetry;
        this.elapsedTime = new ElapsedTime();
    }

    /**
     * Updates the intake process by recording the color of balls entering the slots.
     * @param hue The current hue value from the color sensor.
     */
    public void scanAndRecord(double hue) {
        if (currentSlot >= 3) {
            telemetry.addData("Sorter Status", "FULL: " + Arrays.toString(recordedColors));
            return;
        }

        moveToSlot(currentSlot);

        BallColor detected = detectColor(hue);

        if ((detected == BallColor.PURPLE || detected == BallColor.GREEN) && !ballDetected) {
            recordedColors[currentSlot] = detected;
            currentSlot++;
            ballDetected = true;
        } else if (detected == BallColor.NONE) {
            ballDetected = false;
        }

        telemetry.addData("Recording Slot", currentSlot + 1);
        telemetry.addData("Sensor Color", detected);
        telemetry.addData("Memory", Arrays.toString(recordedColors));
    }

    /**
     * Moves to the slot containing the specified color, prioritizing unused slots.
     * @param target The color to look for.
     */
    public void sortToColor(BallColor target) {
        telemetry.addData("Search Target", target);
        for (int i = 0; i < 3; i++) {
            telemetry.addData("Check Slot " + i, "Color: " + recordedColors[i] + ", Used: " + slotUsed[i]);
            if (recordedColors[i] == target && !slotUsed[i]) {
                moveToSlot(i);
                slotUsed[i] = true;
                telemetry.addData("Sort Result", "FOUND in slot " + (i + 1));
                return;
            }
        }
        telemetry.addData("Sort Result", "NOT FOUND: " + target);
    }

    /**
     * Moves both servos to the given slot position.
     * sorter2 is physically mirrored so it receives the complement (1.0 - pos),
     * plus a small trim offset to prevent fighting at the target position.
     */
    public void moveToSlot(int slot) {
        if (slot == lastCommandedSlot) return;
        lastCommandedSlot = slot;
        double pos;
        switch (slot) {
            case 0: pos = POS_1; break;
            case 1: pos = POS_2; break;
            case 2: pos = POS_3; break;
            default: return;
        }
        hw.sorter1.setPosition(pos);
        // hw.sorter2.setPosition(clamp(1.0 - pos + SERVO2_OFFSET));
    }

    private static double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    private static BallColor detectColor(double hue) {
        if (hue > 200 && hue < 375) return BallColor.PURPLE;
        if (hue > 65  && hue < 175) return BallColor.GREEN;
        return BallColor.NONE;
    }

    public BallColor[] getRecordedColors() {
        return recordedColors;
    }

    public int getCurrentSlot() {
        return currentSlot;
    }

    public void reset() {
        recordedColors = new BallColor[]{ BallColor.NONE, BallColor.NONE, BallColor.NONE };
        slotUsed = new boolean[]{ false, false, false };
        currentSlot = 0;
        ballDetected = false;
        lastCommandedSlot = -1;
    }

    public void transfer() {
        // 0.15
        hw.flipper.setPosition(0.15);
        elapsedTime.reset();
        while (elapsedTime.seconds() < 2.0 && hw.ds.getDistance(DistanceUnit.MM) < 100) {
            hw.intake.set(0.8);
        }
        hw.intake.set(0);
        hw.flipper.setPosition(0);
        try { Thread.sleep(100); } catch (InterruptedException e) {}
    }
}