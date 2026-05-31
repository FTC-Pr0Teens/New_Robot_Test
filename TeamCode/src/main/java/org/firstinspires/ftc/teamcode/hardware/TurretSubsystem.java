package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class TurretSubsystem {

    private final DcMotorEx turret;
    private final Servo hood;

    private final Hardware hw;
    private Follower follower;

    // ---------------- TURRET PD CONTROL ----------------
    public static double TICKS_PER_DEGREE = 1.6122; 
    public static double kP = 0.02; 
    public static double kI = 0.0;
    public static double kD = 0.0005;
    
    private double lastError = 0;
    private double totalError = 0;
    private double manualTargetAngle = 0;
    private double lastCalculatedTarget = 0; 
    private double lastTargetFieldAngle = 0; // Added for debugging
    private boolean useManualTarget = false;
    
    private final double ANGLE_TOLERANCE = 4.0; 
    private final double MAX_POWER = 0.25; 

    // --- MECHANICAL LIMITS ---
    public static double MIN_ANGLE = -280.0;
    public static double MAX_ANGLE = 720.0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    // ---------------- HOOD / SHOOTER ----------------
    private final double HOOD_MIN = 0.36;
    private final double HOOD_MAX = 0.75;
    
    private final double MIN_DISTANCE_INCHES = 12.0;
    private final double MAX_DISTANCE_INCHES = 80.0;

    private final double MIN_RPM = 2000;
    private final double MAX_RPM = 4000;
    private double shootRPM = MIN_RPM;

    // ---------------- GOAL POSITION (INCHES) ----------------
    private double goalX = 0; 
    private double goalY = 0; 

    public TurretSubsystem(HardwareMap hwMap) {
        this.hw = Hardware.getInstance(hwMap);
        
        turret = hw.turret;
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        hood = hw.hood;
        loopTimer.reset();
    }

    public void setFollower(Follower follower) {
        this.follower = follower;
    }

    public void setkP(double newkP) { kP = newkP; }
    public void setkD(double newkD) { kD = newkD; }
    public double getShootRPM() { return shootRPM; }

    public void update() {
        updateInternal(null);
    }

    public void update(Double tx) {
        updateInternal(tx);
    }

    /**
     * Unified update logic using Pedro Heading (90=Straight, 0=Right)
     */
    private void updateInternal(Double tx) {
        double deltaTime = loopTimer.seconds();
        deltaTime = Math.max(deltaTime, 0.01);
        loopTimer.reset();

        double goalAngleDeg;

        if (useManualTarget) {
            goalAngleDeg = manualTargetAngle;
        } else {
            if (follower == null) return;

            Pose robotPose = follower.getPose();
            if (robotPose == null) return;
            
            // PEDRO MATH: X is Side-to-Side (cos), Y is Forward-Backward (sin)
            double dx = goalX - robotPose.getX();
            double dy = goalY - robotPose.getY();

            // Field angle where 0 is Right, 90 is Forward
            double targetAngleField = Math.atan2(dy, dx); 
            lastTargetFieldAngle = Math.toDegrees(targetAngleField);
            
            // Calculate error. Since Pedro Heading 90->0 is Right (+), 
            // and our turret is + Right, the target is (Robot Heading - Target Field Angle)
            double targetAngleRobot = normalizeRadians(robotPose.getHeading() - targetAngleField);
            
            goalAngleDeg = Math.toDegrees(targetAngleRobot);

            if (tx != null) {
                goalAngleDeg += tx; // tx positive is Right
            }

            lastCalculatedTarget = goalAngleDeg;
            
            // ---------------- HOOD & SHOOTER ----------------
            double distance = Math.hypot(dx, dy); 
            distance = Range.clip(distance, MIN_DISTANCE_INCHES, MAX_DISTANCE_INCHES);

            double normalized = (distance - MIN_DISTANCE_INCHES) / (MAX_DISTANCE_INCHES - MIN_DISTANCE_INCHES);
            double hoodPos = HOOD_MIN + Math.pow(normalized, 3.0) * (HOOD_MAX - HOOD_MIN);
            hood.setPosition(Range.clip(hoodPos, HOOD_MIN, HOOD_MAX));

            shootRPM = MIN_RPM + normalized * 1000; 
            shootRPM = Range.clip(shootRPM, MIN_RPM, MAX_RPM);
        }

        // Apply Mechanical Limits
        goalAngleDeg = Range.clip(goalAngleDeg, MIN_ANGLE, MAX_ANGLE);

        double currentAngleDeg = getTurretAngleDegrees();
        double error = wrapDegrees(goalAngleDeg - currentAngleDeg);
        
        if (Math.abs(error) < 5) totalError += error * deltaTime;
        else totalError = 0;
        
        double dTerm = (error - lastError) / deltaTime * kD;
        double power = (Math.abs(error) < ANGLE_TOLERANCE) ? 0 : Range.clip(error * kP + totalError * kI + dTerm, -MAX_POWER, MAX_POWER);

        turret.setPower(power);
        lastError = error;
    }

    private double getTurretAngleDegrees() {
        return (double)turret.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2*Math.PI;
        while (angle < -Math.PI) angle += 2*Math.PI;
        return angle;
    }

    private double wrapDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void setGoalPosition(double xInches, double yInches) {
        goalX = xInches;
        goalY = yInches;
        useManualTarget = false;
    }

    public void setTargetAngle(double angleDeg) {
        manualTargetAngle = angleDeg;
        useManualTarget = true;
    }
    
    public void resetEncoder() {
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        manualTargetAngle = 0;
        lastError = 0;
        totalError = 0;
    }
    
    public void setManualPower(double power) { turret.setPower(power); }
    public double getCurrentAngle() { return getTurretAngleDegrees(); }
    public double getTargetAngle() { return useManualTarget ? manualTargetAngle : lastCalculatedTarget; }
    public double getTargetFieldAngle() { return lastTargetFieldAngle; }
    public double getError() { return wrapDegrees(getTargetAngle() - getCurrentAngle()); }
    public void lockToTag(double bearing) { update(bearing); }
}
