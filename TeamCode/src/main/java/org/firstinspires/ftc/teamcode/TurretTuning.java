package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.TurretSubsystem;

@TeleOp(name="Turret Tuning", group="Tuning")
public class TurretTuning extends OpMode {
    private TurretSubsystem turret;
    
    private double targetAngle = 0;
    
    private int selection = 0; // 0: Angle, 1: kP, 2: kI, 3: kD, 4: TicksPerDegree
    private boolean lastUp = false, lastDown = false;
    private boolean lastRB = false, lastLB = false;

    @Override
    public void init() {
        turret = new TurretSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        // Selection
        if (gamepad1.dpad_up && !lastUp) selection = (selection + 4) % 5;
        if (gamepad1.dpad_down && !lastDown) selection = (selection + 1) % 5;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        // Adjustment
        double step;
        switch(selection) {
            case 0: step = 5.0; break;
            case 1: step = 0.001; break;
            case 2: step = 0.0001; break;
            case 3: step = 0.0005; break; // Changed to be unique and useful
            case 4: step = 0.1; break;
            default: step = 0;
        }

        if (gamepad1.right_bumper && !lastRB) modify(step);
        if (gamepad1.left_bumper && !lastLB) modify(-step);
        lastRB = gamepad1.right_bumper;
        lastLB = gamepad1.left_bumper;

        if (gamepad1.a) turret.resetEncoder();

        turret.setTargetAngle(targetAngle);
        turret.update();

        String selName = "";
        switch(selection) {
            case 0: selName = "Target Angle"; break;
            case 1: selName = "kP"; break;
            case 2: selName = "kI"; break;
            case 3: selName = "kD"; break;
            case 4: selName = "TicksPerDegree"; break;
        }

        telemetry.addData("Selection", selName);
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("kP", TurretSubsystem.kP);
        telemetry.addData("kI", TurretSubsystem.kI);
        telemetry.addData("kD", TurretSubsystem.kD);
        telemetry.addData("Ticks Per Degree", TurretSubsystem.TICKS_PER_DEGREE);
        telemetry.addLine();
        telemetry.addData("Current Angle", "%.2f", turret.getCurrentAngle());
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
