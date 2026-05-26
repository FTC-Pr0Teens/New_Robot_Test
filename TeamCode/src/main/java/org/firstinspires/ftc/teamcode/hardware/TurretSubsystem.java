package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TurretSubsystem {
    private final Hardware hw;

    // --- TUNABLE CONSTANTS ---
    public static double TICKS_PER_DEGREE = 1.6122; 
    
    // Software PD Control Constants
    public static double kP = 0.035; 
    public static double kI = 0.0;
    public static double kD = 0.001;
    
    private double lastError = 0;
    private double totalError = 0;
    private final double ANGLE_TOLERANCE = 5; // degrees - increased to reduce "fixation"
    private final double MAX_AUTO_POWER = 0.4; // reduced to prevent losing tag during rapid motion

    private final ElapsedTime loopTimer = new ElapsedTime();

    // --- MECHANICAL LIMITS ---
    public static double MIN_ANGLE = -280.0;
    public static double MAX_ANGLE = 720.0;

    private double targetAngle = 0.0; 
    private boolean autoMode = true;

    public TurretSubsystem(HardwareMap hwMap) {
        this.hw = Hardware.getInstance(hwMap);
        hw.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        loopTimer.reset();
    }

    /**
     * Updates the turret power using software PID control.
     */
    public void update() {
        double deltaTime = loopTimer.seconds();
        if (deltaTime < 0.001) deltaTime = 0.001; 
        loopTimer.reset();

        double currentAngle = getCurrentAngle();
        double power;

        if (autoMode) {
            double constrainedTarget = Range.clip(targetAngle, MIN_ANGLE, MAX_ANGLE);
            double error = constrainedTarget - currentAngle;
            
            // Integral calculation (for completeness, even if kI=0)
            if (Math.abs(error) < 5) totalError += error * deltaTime;
            else totalError = 0;

            // Derivative calculation
            double dTerm = (error - lastError) / deltaTime * kD;
            
            // Power calculation (PID)
            if (Math.abs(error) < ANGLE_TOLERANCE) {
                power = 0;
                totalError = 0;
            } else {
                power = Range.clip(error * kP + totalError * kI + dTerm, -MAX_AUTO_POWER, MAX_AUTO_POWER);
            }
            
            lastError = error;
        } else {
            // Manual mode power check for limits
            power = hw.turret.getPower();
            if ((currentAngle <= MIN_ANGLE && power < 0) || (currentAngle >= MAX_ANGLE && power > 0)) {
                power = 0;
            }
            lastError = 0;
            totalError = 0;
        }

        hw.turret.setPower(power);
    }

    public void setTargetAngle(double degrees) {
        this.targetAngle = degrees;
        this.autoMode = true;
    }

    public void lockToTag(double bearing) {
        setTargetAngle(getCurrentAngle() + bearing);
    }

    public void lockToFieldAngle(double fieldTargetAngle, double robotHeading) {
        double relativeAngle = AngleUnit.normalizeDegrees(fieldTargetAngle - robotHeading);
        setTargetAngle(relativeAngle);
    }

    public void setManualPower(double power) {
        this.autoMode = false;
        hw.turret.setPower(power);
    }

    public double getCurrentAngle() {
        return hw.turret.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public void resetEncoder() {
        hw.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = 0;
        lastError = 0;
        totalError = 0;
        autoMode = true;
    }

    public boolean isOnTarget() {
        return Math.abs(targetAngle - getCurrentAngle()) < ANGLE_TOLERANCE;
    }
    
    public double getRequestedPower() {
        return hw.turret.getPower();
    }
    
    public boolean isAutoMode() {
        return autoMode;
    }
}
