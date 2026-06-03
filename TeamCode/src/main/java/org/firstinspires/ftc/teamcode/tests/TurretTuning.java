package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
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
    private org.firstinspires.ftc.teamcode.hardware.Hardware hw;
    
    private double targetAngle = 0;
    private boolean autoAlignEnabled = false;
    private boolean lastX = false;
    
    private int selection = 0; // 0: Angle, 1: kP, 2: kI, 3: kD, 4: TicksPerDegree, 5: Alliance, 6: RPM, 7: Hood
    private boolean lastUp = false, lastDown = false;
    private boolean lastRB = false, lastLB = false;
    private boolean lastY = false, lastB = false;

    private boolean isCalibrating = false;
    private int startTicks = 0;

    private TurretSubsystem.Alliance alliance = TurretSubsystem.Alliance.BLUE;
    private double tunedRPM = 3000;
    private double tunedHood = 0.5;
    private int tuningMode = 0; // 0: Auto, 1: Tune RPM (Hood Auto), 2: Tune Hood (RPM Auto)

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

        turret = new TurretSubsystem(hardwareMap);
        turret.setFollower(follower);
        hw = org.firstinspires.ftc.teamcode.hardware.Hardware.getInstance(hardwareMap);
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
        if (gamepad1.dpad_up && !lastUp) selection = (selection + 7) % 8;
        if (gamepad1.dpad_down && !lastDown) selection = (selection + 1) % 8;
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
            case 6: step = 50; break; // RPM step
            case 7: step = 0.01; break; // Hood step
            default: step = 0;
        }

        if (gamepad1.right_bumper && !lastRB) modify(step);
        if (gamepad1.left_bumper && !lastLB) modify(-step);
        lastRB = gamepad1.right_bumper;
        lastLB = gamepad1.left_bumper;

        // Intake control with A
        if (gamepad1.a && !gamepad1.start) {
            hw.intake.setPower(1.0);
        } else {
            hw.intake.setPower(0);
        }

        // Keep flipper (clipper) up
        hw.flipper.setPosition(0);

        // Tuning Mode Toggle (Mode 1 / Mode 2)
        if (gamepad1.start) {
             if (gamepad1.a) tuningMode = 0;
             if (gamepad1.x) tuningMode = 1; // Tune RPM
             if (gamepad1.y) tuningMode = 2; // Tune Hood
        }

        if (gamepad1.back) turret.resetEncoder();

        // 180 Degree Calibration Logic
        if (gamepad1.y && !lastY) {
            isCalibrating = true;
            startTicks = turret.getTicks();
        }
        lastY = gamepad1.y;

        if (gamepad1.b && !lastB && isCalibrating) {
            int endTicks = turret.getTicks();
            int deltaTicks = Math.abs(endTicks - startTicks);
            TurretSubsystem.TICKS_PER_DEGREE = deltaTicks / 180.0;
            isCalibrating = false;
        }
        lastB = gamepad1.b;

        if (autoAlignEnabled) {
            turret.setAlliance(alliance);
            if (tuningMode == 1) {
                turret.setManualRPM(tunedRPM);
            } else if (tuningMode == 2) {
                turret.setManualHood(tunedHood);
            } else {
                turret.disableManualShooter();
            }
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
            case 5: selName = "Alliance"; break;
            case 6: selName = "Manual RPM"; break;
            case 7: selName = "Manual Hood"; break;
            default: selName = "Unknown"; break;
        }

        telemetry.addData("MODE", autoAlignEnabled ? "AUTO-ALIGN" : "MANUAL TUNING");
        telemetry.addData("SHOOTER MODE", tuningMode == 0 ? "AUTO" : (tuningMode == 1 ? "TUNE RPM (Hood Auto)" : "TUNE HOOD (RPM Auto)"));
        telemetry.addLine("X to toggle Auto-Align | Start+A/X/Y for Shooter Mode");
        telemetry.addData("Distance to Goal", "%.2f inches", turret.getDistance());
        telemetry.addLine();
        telemetry.addData("Selection", selName);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Tuned RPM", tunedRPM);
        telemetry.addData("Tuned Hood", tunedHood);
        telemetry.addLine();
        telemetry.addData("Manual Target", targetAngle);
        telemetry.addData("kP", TurretSubsystem.kP);
        telemetry.addData("kI", TurretSubsystem.kI);
        telemetry.addData("kD", TurretSubsystem.kD);
        telemetry.addData("Ticks Per Degree", TurretSubsystem.TICKS_PER_DEGREE);
        telemetry.addLine();
        telemetry.addData("Current Angle", "%.2f", turret.getCurrentAngle());
        
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
            case 5: alliance = (alliance == TurretSubsystem.Alliance.BLUE) ? TurretSubsystem.Alliance.RED : TurretSubsystem.Alliance.BLUE; break;
            case 6: tunedRPM += step; break;
            case 7: tunedHood += step; break;
        }
    }
}
