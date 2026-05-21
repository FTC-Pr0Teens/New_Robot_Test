package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class Sorter {

    public enum BallColor { PURPLE, GREEN, NONE }

    private static final double POS_1 = 0.19;
    private static final double POS_2 = 0.57;
    private static final double POS_3 = 0.93;

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
     * @param alpha The brightness/closeness value from the sensor.
     */
    public void scanAndRecord(double hue, double alpha) {
        if (currentSlot >= 3) {
            telemetry.addData("Sorter Status", "FULL: " + Arrays.toString(recordedColors));
            return;
        }

        // Ensure sorter is at the current slot to receive the ball
        moveToSlot(currentSlot);

        // Only classify if something is actually in front of the sensor (Alpha threshold)
        // If alpha is too low, treat it as NONE (no ball present)
        BallColor detected = (alpha > 0.5) ? detectColor(hue) : BallColor.NONE;

        if ((detected == BallColor.PURPLE || detected == BallColor.GREEN) && !ballDetected) {
            recordedColors[currentSlot] = detected;
            currentSlot++;
            ballDetected = true;
        } else if (detected == BallColor.NONE) {
            // Ball has passed or no ball present
            ballDetected = false;
        }

        telemetry.addData("Recording Slot", currentSlot + 1);
        telemetry.addData("Sensor Color", detected);
        telemetry.addData("Sensor Alpha", "%.3f", alpha);
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
        hw.sorter2.setPosition(clamp(1.0 - pos + SERVO2_OFFSET));
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

    /**
     * Call this when reversing the intake to clear the last recorded ball
     * as it passes the sensor in reverse.
     */
    public void scanAndRemove(double hue, double alpha) {
        if (currentSlot <= 0) return;

        // Ensure sorter is at the last recorded slot to let the ball out
        moveToSlot(currentSlot - 1);

        BallColor detected = (alpha > 0.5) ? detectColor(hue) : BallColor.NONE;

        // If we see a ball and we haven't already marked it as removed for this "pass"
        if ((detected == BallColor.PURPLE || detected == BallColor.GREEN) && !ballDetected) {
            currentSlot--;
            recordedColors[currentSlot] = BallColor.NONE;
            slotUsed[currentSlot] = false;
            ballDetected = true;
        } else if (detected == BallColor.NONE) {
            ballDetected = false;
        }

        telemetry.addData("Outtaking Slot", currentSlot + 1);
        telemetry.addData("Memory", Arrays.toString(recordedColors));
    }

    public void reset() {
        recordedColors = new BallColor[]{ BallColor.NONE, BallColor.NONE, BallColor.NONE };
        slotUsed = new boolean[]{ false, false, false };
        currentSlot = 0;
        ballDetected = false;
        lastCommandedSlot = -1;
    }

    /**
     * Executes the release of the ball by moving the flipper and running the intake.
     * Keeps the shooter PID alive during the process to ensure speed is maintained.
     */
    public void transfer(ShooterSubsystem shooter) {
        // Step 1: Wait for sorter servos to reach their destination before flipping
        try { Thread.sleep(400); } catch (InterruptedException e) {}

        // Step 2: Clear memory for the slot we are about to shoot
        // This ensures the list stays accurate as balls exit
        if (lastCommandedSlot != -1) {
            recordedColors[lastCommandedSlot] = BallColor.NONE;
            slotUsed[lastCommandedSlot] = false;
            if (currentSlot > 0) currentSlot--;
        }

        // Step 3: Move flipper UP to the scoring position
        hw.flipper.setPosition(0); 
        
        // Step 4: Run intake for a set time to push the ball out
        elapsedTime.reset();
        while (elapsedTime.seconds() < 1.1) {
            hw.intake.setPower(1.0);
            // CRITICAL: Keep the shooter PID running while the ball is being transferred
            if (shooter != null) shooter.update();
        }
        hw.intake.setPower(0);
        
        // Step 5: Return flipper to LOW position so next ball can enter slots
        hw.flipper.setPosition(0.15); 
        try { Thread.sleep(150); } catch (InterruptedException e) {}
    }
}