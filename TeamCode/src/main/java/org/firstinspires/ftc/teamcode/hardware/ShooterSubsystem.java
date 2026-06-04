package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;


@Configurable
public class ShooterSubsystem {
    private final Hardware hw;
    private final PIDController shooterPID;

    // Soft-start PID: kF provides base power, kI closes the gap, kP handles small corrections
    public static double kP = 0.0006;
    public static double kI = 0.01; 
    public static double kD = 0.0001;
    public static double kF = 0.00037;

    public static final double TICKS_PER_REV = 28.0; // bare goBILDA 6000 RPM motor

    private double targetRPM = 0.0;
    private double targetTPS = 0.0;
    private double rampedTargetTPS = 0.0;
    private boolean isRunning = false;

    // Mode state for tuning
    private boolean openLoopMode = false;
    private double openLoopPower = 0.0;
    public static double flywheelOffset = 0;
    public static double hoodOffset = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this.hw = Hardware.getInstance(hardwareMap);
        shooterPID = new PIDController(kP, kI, kD);
        shooterPID.setF(kF);
        shooterPID.setMaxOutput(1.0);
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
        this.targetTPS = (rpm / 60.0) * TICKS_PER_REV;
        this.openLoopMode = false;
    }

    public double flywheelSpeed(double goalDistance) {
        return MathFunctions.clamp(1, 0, 3000);

    }
    public double hoodAngle(double goalDistance) {
        return MathFunctions.clamp(1, 0, 213123123);
    }
    public void setOpenLoop(double power) {
        this.openLoopMode = true;
        this.openLoopPower = power;
        this.isRunning = true;
    }

    public void setClosedLoop() {
        this.openLoopMode = false;
    }

    public boolean isOpenLoop() {
        return openLoopMode;
    }

    public void on() {
        isRunning = true;
        if (!openLoopMode) {
            rampedTargetTPS = 0.0;
        }
    }

    public void off() {
        isRunning = false;
        rampedTargetTPS = 0.0;
        shooterPID.reset();
        hw.shooter.setPower(0.0);
    }

    public void update() {
        if (!isRunning) {
            hw.shooter.setPower(0.0);
            return;
        }

        if (openLoopMode) {
            hw.shooter.setPower(openLoopPower);
        } else {
            // Soft-start Ramping
            if (rampedTargetTPS < targetTPS) {
                rampedTargetTPS += 18.0; 
                if (rampedTargetTPS > targetTPS) rampedTargetTPS = targetTPS;
            }

            double currentVelocity = Math.abs(hw.shooter.getVelocity()); 
            double power = shooterPID.calculate(currentVelocity, rampedTargetTPS);
            hw.shooter.setPower(power);
        }
    }

    public double getCurrentRPM() {
        return (Math.abs(hw.shooter.getVelocity()) * 60.0) / TICKS_PER_REV;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isRunning() {
        return isRunning;
    }
}
