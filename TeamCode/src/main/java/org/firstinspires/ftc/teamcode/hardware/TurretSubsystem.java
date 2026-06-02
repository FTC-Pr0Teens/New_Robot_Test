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
    public static double TICKS_PER_DEGREE = 4.3667;
    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.0005;
    
    private double lastError = 0;
    private double totalError = 0;
    private double manualTargetAngle = 0;
    private double lastCalculatedTarget = 0; 
    private double lastTargetFieldAngle = 0; 
    private double lastCalculatedDistance = 0;
    private double lastRequestedPower = 0;
    private boolean useManualTarget = false;

    private final double ANGLE_TOLERANCE = 4.0;
    private final double MAX_POWER = 0.25;

    // --- MECHANICAL LIMITS ---
    public static double MIN_ANGLE = -120.0;
    public static double MAX_ANGLE = 360.0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    // ---------------- HOOD / SHOOTER ----------------
    private final double HOOD_MIN = 0.36;
    private final double HOOD_MAX = 0.75;
    private final double MIN_DISTANCE_METERS = 0.3;
    private final double MAX_DISTANCE_METERS = 2.0;

    private final double MIN_RPM = 2000;
    private final double MAX_RPM = 3000;
    private double shootRPM = MIN_RPM;
    private double calculatedHoodPos = 0.5;

    private final double INCHES_TO_METERS = 0.0254;

    public enum Alliance { BLUE, RED }
    private Alliance alliance = Alliance.BLUE;

    // ---------------- GOAL POSITION (INCHES) ----------------
    private double goalX = 0.0;
    private double goalY = 144.0; 

    private boolean manualHoodEnabled = false;
    private boolean manualRPMEnabled = false;
    private double manualHoodPos = 0.5;
    private double manualRPM = 3000;

    public TurretSubsystem(HardwareMap hwMap) {
        this.hw = Hardware.getInstance(hwMap);
        turret = hw.turret;
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hood = hw.hood;
        loopTimer.reset();
    }

    public void setFollower(Follower follower) { this.follower = follower; }
    public void setkP(double newkP) { kP = newkP; }
    public void setkD(double newkD) { kD = newkD; }
    public double getShootRPM() { return shootRPM; }

    public void update() {
        double deltaTime = loopTimer.seconds();
        if (deltaTime < 0.001) deltaTime = 0.001; 
        loopTimer.reset();

        double currentAngleDeg = getTurretAngleDegrees();
        double targetAngle;

        if (useManualTarget) {
            targetAngle = manualTargetAngle;
        } else {
            if (follower == null) return;
            Pose robotPose = follower.getPose();
            if (robotPose == null) return;
            
            double dx = goalX - robotPose.getX();
            double dy = goalY - robotPose.getY();

            // Field Angle Calculation: 0=Right (Pos X), 90=Straight (Pos Y)
            double targetAngleFieldRad = Math.atan2(dy, dx);
            lastTargetFieldAngle = Math.toDegrees(targetAngleFieldRad);
            
            // Relative Angle: RobotHeading - FieldAngle
            double baseRelativeDeg = Math.toDegrees(normalizeRadians(robotPose.getHeading() - targetAngleFieldRad));
            
            // Intelligent Wrapping: Find version of target closest to current position within limits
            targetAngle = getBestReachableVersion(baseRelativeDeg, currentAngleDeg);
            lastCalculatedTarget = targetAngle;

            // ---------------- HOOD & SHOOTER LOGIC ----------------
            double distanceInches = Math.hypot(dx, dy);
            lastCalculatedDistance = distanceInches;

            double distanceMeters = (distanceInches * INCHES_TO_METERS) * 0.85;
            double normalized = Range.clip((distanceMeters - MIN_DISTANCE_METERS) / (MAX_DISTANCE_METERS - MIN_DISTANCE_METERS), 0, 1);

            if (!manualHoodEnabled) {
                hood.setPosition(Range.clip(HOOD_MIN + normalized * (HOOD_MAX - HOOD_MIN), HOOD_MIN, HOOD_MAX));
            } else {
                hood.setPosition(manualHoodPos);
            }

            if (manualRPMEnabled) {
                shootRPM = manualRPM;
            } else {
                shootRPM = MIN_RPM + (normalized * (MAX_RPM - MIN_RPM));
            }
        }

        // Final PID Control
        double error = targetAngle - currentAngleDeg; 
        
        if (Math.abs(error) < 5) totalError += error * deltaTime;
        else totalError = 0;
        
        double dTerm = (error - lastError) / deltaTime * kD;
        double power = (Math.abs(error) < ANGLE_TOLERANCE) ? 0 : Range.clip(error * kP + totalError * kI + dTerm, -MAX_POWER, MAX_POWER);

        lastRequestedPower = power;
        turret.setPower(power);
        lastError = error;
    }

    /**
     * Logic to find the multiple of 360 that is within limits and closest to current position.
     */
    private double getBestReachableVersion(double target, double current) {
        double[] candidates = {target - 720, target - 360, target, target + 360, target + 720};
        double best = target;
        double minDiff = Double.MAX_VALUE;
        boolean foundValid = false;

        for (double v : candidates) {
            if (v >= MIN_ANGLE && v <= MAX_ANGLE) {
                double diff = Math.abs(v - current);
                if (diff < minDiff) {
                    minDiff = diff;
                    best = v;
                    foundValid = true;
                }
            }
        }
        return foundValid ? best : Range.clip(target, MIN_ANGLE, MAX_ANGLE);
    }

    private double getTurretAngleDegrees() { return (double)turret.getCurrentPosition() / TICKS_PER_DEGREE; }
    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2*Math.PI;
        while (angle < -Math.PI) angle += 2*Math.PI;
        return angle;
    }
    public void setGoalPosition(double xInches, double yInches) {
        goalX = xInches; goalY = yInches; useManualTarget = false;
    }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        if (alliance == Alliance.BLUE) {
            setGoalPosition(72.0, 144.0);
        } else {
            setGoalPosition(72.0, 0.0);
        }
    }
    public void setManualHood(double pos) {
        this.manualHoodPos = pos;
        this.manualHoodEnabled = true;
        hw.hood.setPosition(pos);
    }
    public void disableManualHood() {
        this.manualHoodEnabled = false;
    }
    public void setManualRPM(double rpm) {
        this.manualRPM = rpm;
        this.manualRPMEnabled = true;
    }
    public void disableManualRPM() {
        this.manualRPMEnabled = false;
    }
    public void disableManualShooter() {
        this.manualHoodEnabled = false;
        this.manualRPMEnabled = false;
    }
    public void setTargetAngle(double angleDeg) {
        manualTargetAngle = angleDeg; useManualTarget = true;
    }
    public void resetEncoder() {
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        manualTargetAngle = 0; lastError = 0; totalError = 0;
    }
    public void setManualPower(double power) { this.useManualTarget = true; this.lastRequestedPower = power; turret.setPower(power); }
    public double getCurrentAngle() { return getTurretAngleDegrees(); }
    public int getTicks() { return turret.getCurrentPosition(); }
    public double getTargetAngle() { return useManualTarget ? manualTargetAngle : lastCalculatedTarget; }
    public double getTargetFieldAngle() { return lastTargetFieldAngle; }
    public double getDistance() { return lastCalculatedDistance; }
    public double getError() { return getTargetAngle() - getCurrentAngle(); }
    public double getRequestedPower() { return lastRequestedPower; }
}
