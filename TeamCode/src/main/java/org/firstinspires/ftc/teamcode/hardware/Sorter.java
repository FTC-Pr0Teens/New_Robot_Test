package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Arrays;

public class Sorter {

    public enum BallColor { PURPLE, GREEN, NONE }

    private static final double POS_1 = 0.17;
    private static final double POS_2 = 0.54;
    private static final double POS_3 = 0.86;
    private static final double SERVO2_OFFSET = 0.05;

    private Hardware hw;
    private Telemetry telemetry;
    private int lastCommandedSlot = -1;

    private BallColor[] recordedColors = { BallColor.NONE, BallColor.NONE, BallColor.NONE };
    private boolean[] slotUsed = { false, false, false };

    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime recordCooldown = new ElapsedTime();
    
    private enum TransferState { IDLE, WAITING_FOR_SERVO, INTAKING, FLIPPING_DOWN }
    private TransferState currentTransferState = TransferState.IDLE;

    public Sorter(HardwareMap hwmap, Telemetry telemetry) {
        hw = Hardware.getInstance(hwmap);
        this.telemetry = telemetry;
    }

    /**
     * Finds the first empty physical slot and waits there.
     * Robust recording: records color, then waits for a cooldown before moving again.
     */
    public void scanAndRecord(double hue, double alpha) {
        if (currentTransferState != TransferState.IDLE) return;

        // Find the first empty hole
        int target = -1;
        for (int i = 0; i < 3; i++) {
            if (recordedColors[i] == BallColor.NONE) {
                target = i;
                break;
            }
        }

        if (target == -1) return; // Full

        // Always move to the target hole
        moveToSlot(target);

        // ENFORCED PAUSE: Ignore sensor for 300ms after a recording to prevent noise
        if (recordCooldown.milliseconds() < 300) return;

        // Detect ball with high threshold (0.65)
        if (alpha > 0.65) {
            BallColor detected = detectColor(hue);
            if (detected != BallColor.NONE) {
                recordedColors[target] = detected;
                slotUsed[target] = false;
                recordCooldown.reset(); // Start the 300ms blind period
            }
        }
    }

    /**
     * Efficient search: searches from the closest end of the robot.
     */
    public boolean sortToColor(BallColor target) {
        if (currentTransferState != TransferState.IDLE) return false;

        int current = (lastCommandedSlot == -1) ? 0 : lastCommandedSlot;
        
        // SearchOrder: current, then closest neighbors
        int[] searchOrder;
        if (current == 0)      searchOrder = new int[]{0, 1, 2};
        else if (current == 1) searchOrder = new int[]{1, 0, 2};
        else                   searchOrder = new int[]{2, 1, 0};

        for (int i : searchOrder) {
            if (recordedColors[i] == target && !slotUsed[i]) {
                moveToSlot(i);
                slotUsed[i] = true;
                return true;
            }
        }
        return false;
    }

    public void startTransfer() {
        if (currentTransferState == TransferState.IDLE) {
            currentTransferState = TransferState.WAITING_FOR_SERVO;
            stateTimer.reset();
        }
    }

    public void update(ShooterSubsystem shooter) {
        switch (currentTransferState) {
            case WAITING_FOR_SERVO:
                if (stateTimer.milliseconds() > 400) { 
                    if (lastCommandedSlot != -1) {
                        recordedColors[lastCommandedSlot] = BallColor.NONE;
                        slotUsed[lastCommandedSlot] = false;
                    }
                    hw.flipper.setPosition(0); // UP
                    currentTransferState = TransferState.INTAKING;
                    stateTimer.reset();
                }
                break;
            case INTAKING:
                hw.intake.setPower(1.0);
                if (stateTimer.seconds() > 1.1) {
                    hw.intake.setPower(0);
                    hw.flipper.setPosition(0.15); // DOWN
                    currentTransferState = TransferState.FLIPPING_DOWN;
                    stateTimer.reset();
                }
                break;
            case FLIPPING_DOWN:
                if (stateTimer.milliseconds() > 150) {
                    currentTransferState = TransferState.IDLE;
                }
                break;
            case IDLE:
                break;
        }
        if (currentTransferState != TransferState.IDLE && shooter != null) {
            shooter.update();
        }
    }

    public boolean isBusy() {
        return currentTransferState != TransferState.IDLE;
    }

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
        hw.sorter2.setPosition(Math.max(0, Math.min(1, 1.0 - pos + SERVO2_OFFSET)));
    }

    private BallColor detectColor(double hue) {
        if (hue > 200 && hue < 375) return BallColor.PURPLE;
        if (hue > 65  && hue < 175) return BallColor.GREEN;
        return BallColor.NONE;
    }

    public void scanAndRemove(double alpha) {
        if (currentTransferState != TransferState.IDLE) return;
        
        // Find the last filled slot
        int last = -1;
        for (int i = 2; i >= 0; i--) {
            if (recordedColors[i] != BallColor.NONE) {
                last = i;
                break;
            }
        }
        if (last == -1) return;

        moveToSlot(last);
        
        // Use threshold and cooldown for removal too
        if (alpha > 0.65 && recordCooldown.milliseconds() > 300) {
            recordedColors[last] = BallColor.NONE;
            slotUsed[last] = false;
            recordCooldown.reset();
        }
    }

    public void reset() {
        recordedColors = new BallColor[]{ BallColor.NONE, BallColor.NONE, BallColor.NONE };
        slotUsed = new boolean[]{ false, false, false };
        lastCommandedSlot = -1;
        currentTransferState = TransferState.IDLE;
    }

    public BallColor[] getRecordedColors() {
        return recordedColors;
    }
}
