package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.hardware.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Turret Tuning", group="Tuning")
public class TurretTuning extends OpMode {
    private TurretSubsystem turret;
    private Follower follower;
    
    private double targetAngle = 0;
    private boolean autoAlignEnabled = false;
    private boolean lastX = false;
    
    private int selection = 0; // 0: Angle, 1: kP, 2: kI, 3: kD, 4: TicksPerDegree
    private boolean lastUp = false, lastDown = false;
    private boolean lastRB = false, lastLB = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

        turret = new TurretSubsystem(hardwareMap);
        turret.setFollower(follower);
    }

    @Override
    public void loop() {
        follower.update();

        // Toggle Auto-Align
        if (gamepad1.x && !lastX) {
            autoAlignEnabled = !autoAlignEnabled;
            if (!autoAlignEnabled) {
                // If turning off, set current target as manual target to avoid sudden snap
                targetAngle = turret.getCurrentAngle();
                turret.setTargetAngle(targetAngle);
            }
        }
        lastX = gamepad1.x;

        // Selection
        if (gamepad1.dpad_up && !lastUp) selection = (selection + 4) % 5;
        if (gamepad1.dpad_down && !lastDown) selection = (selection + 1) % 5;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        // Adjustment
        double step;
        switch(selection) {
            case 0: step = 5.0; break;
            case 1: step = 0.001; break; // kP step
            case 2: step = 0.0001; break; // kI step
            case 3: step = 0.00005; break; // kD step
            case 4: step = 0.1; break;
            default: step = 0;
        }

        if (gamepad1.right_bumper && !lastRB) modify(step);
        if (gamepad1.left_bumper && !lastLB) modify(-step);
        lastRB = gamepad1.right_bumper;
        lastLB = gamepad1.left_bumper;

        if (gamepad1.a) turret.resetEncoder();

        if (autoAlignEnabled) {
            // Use the odometry logic built into TurretSubsystem
            // Ensure goal is set (default is 0, 3.0 in subsystem)
            turret.update(); 
        } else {
            // Manual angle testing
            turret.setTargetAngle(targetAngle);
            turret.update();
        }

        String selName;
        switch(selection) {
            case 0: selName = "Target Angle (Manual Mode)"; break;
            case 1: selName = "kP"; break;
            case 2: selName = "kI"; break;
            case 3: selName = "kD"; break;
            case 4: selName = "TicksPerDegree"; break;
            default: selName = "Unknown"; break;
        }

        telemetry.addData("MODE", autoAlignEnabled ? "AUTO-ALIGN (Lock to 0m, 3m)" : "MANUAL TUNING");
        telemetry.addLine("Press X to toggle mode");
        telemetry.addLine();
        telemetry.addData("Selection", selName);
        telemetry.addData("Manual Target", targetAngle);
        telemetry.addData("kP", TurretSubsystem.kP);
        telemetry.addData("kI", TurretSubsystem.kI);
        telemetry.addData("kD", TurretSubsystem.kD);
        telemetry.addData("Ticks Per Degree", TurretSubsystem.TICKS_PER_DEGREE);
        telemetry.addLine();
        telemetry.addData("Current Angle", "%.2f", turret.getCurrentAngle());
        
        Pose pose = follower.getPose();
        telemetry.addData("Robot Pose", "X: %.2f, Y: %.2f, H: %.2f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }

    private void modify(double step) {
        switch(selection) {
            case 0: targetAngle += step; break;
            case 1: TurretSubsystem.kP += step; break;
            case 2: TurretSubsystem.kI += step; break;
            case 3: TurretSubsystem.kD += step; break;
            case 4: TurretSubsystem.TICKS_PER_DEGREE += step; break;
        }
    }
}
