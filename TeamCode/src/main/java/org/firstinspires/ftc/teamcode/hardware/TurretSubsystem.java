package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TurretSubsystem {
    private final Hardware hw;

    // --- TUNABLE CONSTANTS ---
    // 1150 RPM goBilda (13.7:1) has 145.1 ticks per revolution at the output shaft.
    // 4:1 external reduction means 4 motor revs = 1 turret rev.
    // Ticks per degree = (145.1 * 4) / 360 = ~1.6122
    public static double TICKS_PER_DEGREE = 1.6122; 

    // PD Control Constants from Tutorial
    private double kP = 0.035;
    private double kD = 0.001;
    private double lastError = 0;
    private final double ANGLE_TOLERANCE = 0.5; // degrees
    private final double MAX_POWER = 0.4;

    private final ElapsedTime loopTimer = new ElapsedTime();

    // --- MECHANICAL LIMITS ---
    public static double MIN_ANGLE = -430.0;
    public static double MAX_ANGLE = 430.0;

    private double targetAngle = 0.0; 
    private boolean autoMode = true;

    public TurretSubsystem(HardwareMap hwMap) {
        this.hw = Hardware.getInstance(hwMap);
        loopTimer.reset();
    }

    /**
     * Update turret rotation using PD control from tutorial logic.
     * Adapted for robot without odometry.
     */
    public void update() {
        double deltaTime = loopTimer.seconds();
        deltaTime = Math.max(deltaTime, 0.01); // prevent division by zero
        loopTimer.reset();

        double currentAngle = getCurrentAngle();
        double power;

        if (autoMode) {
            // Constrain target within physical limits
            double constrainedTarget = Range.clip(targetAngle, MIN_ANGLE, MAX_ANGLE);
            
            // PD Calculation from Tutorial
            double error = AngleUnit.normalizeDegrees(constrainedTarget - currentAngle);
            double dTerm = (error - lastError) / deltaTime * kD;
            
            power = (Math.abs(error) < ANGLE_TOLERANCE) ? 0 : Range.clip(error * kP + dTerm, -MAX_POWER, MAX_POWER);
            lastError = error;
        } else {
            // Manual power check: Stop if hitting limits
            power = hw.turret.getPower();
            if ((currentAngle <= MIN_ANGLE && power < 0) || (currentAngle >= MAX_ANGLE && power > 0)) {
                power = 0;
            }
            lastError = 0; // Reset error for clean switch back to auto
        }
        
        hw.turret.setPower(power);
    }

    /**
     * Sets the target angle for the turret relative to the robot.
     */
    public void setTargetAngle(double degrees) {
        this.targetAngle = degrees;
        this.autoMode = true;
    }

    /**
     * Aim turret using AprilTag bearing (bearing is tx equivalent).
     * @param bearing Degrees from camera center to tag
     */
    public void lockToTag(double bearing) {
        // bearing is relative to camera center.
        // To point at it, we need to add the offset to our current angle.
        setTargetAngle(getCurrentAngle() + bearing);
    }

    public void lockToFieldAngle(double fieldTargetAngle, double robotHeading) {
        double relativeAngle = AngleUnit.normalizeDegrees(fieldTargetAngle - robotHeading);
        setTargetAngle(relativeAngle);
    }

    public void setManualPower(double power) {
        this.autoMode = false;
        double currentAngle = getCurrentAngle();
        // Prevent manual rotation past limits
        if ((currentAngle <= MIN_ANGLE && power < 0) || (currentAngle >= MAX_ANGLE && power > 0)) {
            hw.turret.setPower(0);
        } else {
            hw.turret.setPower(power);
        }
    }

    public double getCurrentAngle() {
        return hw.turret.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public void setPID(double p, double d) {
        this.kP = p;
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
    }
}
