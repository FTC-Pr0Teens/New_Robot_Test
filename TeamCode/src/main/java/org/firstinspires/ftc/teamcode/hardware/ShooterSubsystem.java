package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;

public class ShooterSubsystem {
    private final Hardware hw;
    private final PIDController shooterPID;

    // Shooter Constants (6000 RPM goBilda) — tune kF first, then kP, then kI/kD
    // Made public so ShooterTuningOp can modify them at runtime
    public static double kP = 0.0012;
    public static double kI = 0.0001;
    public static double kD = 0.0005;
    public static double kF = 0.00045;

    public static final double TICKS_PER_REV = 28.0;

    private double targetRPM = 0.0;
    private double targetTPS = 0.0;
    private boolean isRunning = false;

    // Open-loop mode: bypass PID and drive motor directly with a fixed power
    private boolean openLoopMode = false;
    private double openLoopPower = 0.0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this.hw = Hardware.getInstance(hardwareMap);
        shooterPID = new PIDController(kP, kI, kD);
        shooterPID.setF(kF);
        shooterPID.setMaxOutput(1.0);
    }

    /**
     * Reload PID constants from the static fields.
     * Call this after changing kP/kI/kD/kF during tuning.
     */
    public void reloadConstants() {
        shooterPID.setPID(kP, kI, kD);
        shooterPID.setF(kF);
        shooterPID.reset();
    }

    /** Set the shooter target speed in RPM (closed-loop / PID mode). */
    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
        this.targetTPS = (rpm / 60.0) * TICKS_PER_REV;
    }

    /**
     * Run the shooter in open-loop mode at a fixed power (0.0 – 1.0).
     * Bypasses the PID entirely — useful for finding the right kF value.
     * Observe the steady-state RPM, then set kF = power / targetTPS.
     */
    public void setOpenLoop(double power) {
        openLoopMode = true;
        openLoopPower = power;
        isRunning = true;
    }

    /** Switch back to closed-loop (PID) mode. */
    public void setClosedLoop() {
        openLoopMode = false;
        shooterPID.reset();
    }

    public boolean isOpenLoop() {
        return openLoopMode;
    }

    public void on() {
        isRunning = true;
    }

    public void off() {
        isRunning = false;
        openLoopMode = false;
        shooterPID.reset();
        hw.shooter.set(0.0);
    }

    public void toggle() {
        if (isRunning) off();
        else on();
    }

    /**
     * Main control loop. Call every OpMode loop.
     * Open-loop: drives motor at fixed power.
     * Closed-loop: runs PIDF against target TPS.
     */
    public void update() {
        if (!isRunning) {
            hw.shooter.set(0.0);
            return;
        }
        if (openLoopMode) {
            hw.shooter.set(openLoopPower);
        } else {
            double currentVelocity = hw.shooter.getVelocity(); // ticks/sec
            double power = shooterPID.calculate(currentVelocity, targetTPS);
            hw.shooter.set(power);
        }
    }

    public double getCurrentRPM() {
        return (hw.shooter.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    public double getCurrentTPS() {
        return hw.shooter.getVelocity();
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getTargetTPS() {
        return targetTPS;
    }

    public double getOpenLoopPower() {
        return openLoopPower;
    }

    public boolean isRunning() {
        return isRunning;
    }
}