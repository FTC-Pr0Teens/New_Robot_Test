package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TurretSubsystem {
    private final Hardware hw;

    // --- TUNABLE CONSTANTS ---
    public static double TICKS_PER_DEGREE = 1.6122; 

    // PD Control Constants - Tuned for stability and no shaking
    private double kP = 0.015; 
    private double kI = 0.001;
    private double kD = 0.005;
    
    private double totalError = 0;
    private double lastError = 0;
    private final double ANGLE_TOLERANCE = 5; // degrees
    private final double MAX_POWER = 0.5;

    private final ElapsedTime loopTimer = new ElapsedTime();

    // --- MECHANICAL LIMITS ---
    public static double MIN_ANGLE = -260.0;
    public static double MAX_ANGLE = 260.0;

    private double targetAngle = 0.0; 
    private boolean autoMode = true;

    public TurretSubsystem(HardwareMap hwMap) {
        this.hw = Hardware.getInstance(hwMap);
        loopTimer.reset();
    }

    public void update() {
        double deltaTime = loopTimer.seconds();
        deltaTime = Math.max(deltaTime, 0.001); 
        loopTimer.reset();

        double currentAngle = getCurrentAngle();
        double power;

        if (autoMode) {
            double constrainedTarget = Range.clip(targetAngle, MIN_ANGLE, MAX_ANGLE);
            double error = AngleUnit.normalizeDegrees(constrainedTarget - currentAngle);
            
            // Dampen Integral to prevent windup
            if (Math.abs(error) < 10) totalError += error * deltaTime;
            else totalError = 0;
            
            double dTerm = (error - lastError) / deltaTime * kD;
            
            power = (Math.abs(error) < ANGLE_TOLERANCE) ? 0 : Range.clip(error * kP + totalError * kI + dTerm, -MAX_POWER, MAX_POWER);
            lastError = error;
        } else {
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
        double currentAngle = getCurrentAngle();
        if ((currentAngle <= MIN_ANGLE && power < 0) || (currentAngle >= MAX_ANGLE && power > 0)) {
            hw.turret.setPower(0);
        } else {
            hw.turret.setPower(power);
        }
    }

    public double getCurrentAngle() {
        return hw.turret.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public void setPID(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void resetEncoder() {
        hw.turret.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.turret.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = 0;
        lastError = 0;
        totalError = 0;
    }
}
