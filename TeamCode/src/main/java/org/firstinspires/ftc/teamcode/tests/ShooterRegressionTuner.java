package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.hardware.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Shooter Regression Tuner", group="Tuning")
public class ShooterRegressionTuner extends OpMode {
    private Follower follower;
    private TurretSubsystem turret;
    private ShooterSubsystem shooter;
    private Hardware hw;
    
    private double manualRPM = 3000;
    private double manualHood = 0.5;
    private List<DataPoint> dataPoints = new ArrayList<>();
    
    private boolean lastA1 = false, lastX1 = false, lastB1 = false, lastY1 = false;
    private boolean lastStart2 = false, lastA2 = false;
    
    private int tuningMode = 0; // 0: Auto, 1: Tune RPM (Hood Auto), 2: Tune Hood (RPM Auto)
    private boolean flywheelOn = false;

    private double slope = 0;
    private double intercept = 0;

    private static class DataPoint {
        double distance;
        double rpm;
        DataPoint(double d, double r) { this.distance = d; this.rpm = r; }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));
        follower.update();

        turret = new TurretSubsystem(hardwareMap);
        turret.setFollower(follower);
        // Default to Blue goal
        turret.setGoalPosition(6, 132);

        shooter = new ShooterSubsystem(hardwareMap);
        hw = Hardware.getInstance(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        
        Pose pose = follower.getPose();
        if (pose == null) {
            telemetry.addLine("Odometry Not Init...");
            telemetry.update();
            return;
        }

        // --- GAMEPAD 1: DRIVING & ALLIANCE ---
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        follower.setTeleOpDrive(y, x, rx, true);

        if (gamepad1.x && !lastX1) turret.setAlliance(TurretSubsystem.Alliance.BLUE);
        if (gamepad1.b && !lastB1) turret.setAlliance(TurretSubsystem.Alliance.RED);
        lastX1 = gamepad1.x;
        lastB1 = gamepad1.b;

        // Record Point
        if (gamepad1.a && !lastA1) {
            dataPoints.add(new DataPoint(turret.getDistance(), shooter.getTargetRPM()));
            calculateRegression();
        }
        lastA1 = gamepad1.a;

        if (gamepad1.y && !lastY1) dataPoints.clear();
        lastY1 = gamepad1.y;

        // --- GAMEPAD 2: SHOOTER TUNING ---
        // Toggle Flywheel
        if (gamepad2.a && !lastA2) flywheelOn = !flywheelOn;
        lastA2 = gamepad2.a;

        if (flywheelOn) shooter.on();
        else shooter.off();

        // Tuning Mode
        if (gamepad2.start && !lastStart2) tuningMode = (tuningMode + 1) % 3;
        lastStart2 = gamepad2.start;

        // Adjust Manual Values
        if (tuningMode == 1) { // Tuning RPM
            if (gamepad2.dpad_up) manualRPM += 10;
            if (gamepad2.dpad_down) manualRPM -= 10;
            if (gamepad2.right_bumper) manualRPM += 100;
            if (gamepad2.left_bumper) manualRPM -= 100;
            turret.setManualRPM(manualRPM);
            turret.disableManualHood(); // Hood stays Auto
        } else if (tuningMode == 2) { // Tuning Hood
            if (gamepad2.dpad_up) manualHood += 0.005;
            if (gamepad2.dpad_down) manualHood -= 0.005;
            turret.setManualHood(manualHood);
            turret.disableManualRPM(); // RPM stays Auto
        } else {
            turret.disableManualShooter();
        }

        turret.update();
        shooter.setTargetRPM(turret.getShootRPM());
        shooter.update();

        // Telemetry - EXACTLY what you asked for
        telemetry.addData("Distance to Goal", "%.2f in", turret.getDistance());
        telemetry.addData("Mode", tuningMode == 0 ? "AUTO" : (tuningMode == 1 ? "TUNE RPM" : "TUNE HOOD"));
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Target RPM", "%.0f", shooter.getTargetRPM());
        telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRPM());
        telemetry.addData("Hood Pos", "%.3f", hw.hood.getPosition());
        telemetry.addLine("\nG1: X/B Alliance | A: Record | Y: Clear");
        telemetry.addLine("G2: A: Toggle Flywheel | Start: Mode | Dpad: Adjust");
        telemetry.update();
    }

    private void calculateRegression() {
        int n = dataPoints.size();
        if (n < 2) return;

        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (DataPoint p : dataPoints) {
            sumX += p.distance;
            sumY += p.rpm;
            sumXY += p.distance * p.rpm;
            sumX2 += p.distance * p.distance;
        }

        slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
        intercept = (sumY - slope * sumX) / n;
    }
}
