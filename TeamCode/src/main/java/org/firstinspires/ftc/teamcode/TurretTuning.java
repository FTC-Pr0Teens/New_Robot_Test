package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.TurretSubsystem;

@TeleOp(name="Turret Tuning", group="Tuning")
public class TurretTuning extends OpMode {
    private TurretSubsystem turret;
    
    private double targetAngle = 0;
    
    private int selection = 0; // 0: Angle, 1: kP, 2: TicksPerDegree
    private boolean lastUp = false, lastDown = false;
    private boolean lastRB = false, lastLB = false;

    @Override
    public void init() {
        turret = new TurretSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        // Selection
        if (gamepad1.dpad_up && !lastUp) selection = (selection + 2) % 3;
        if (gamepad1.dpad_down && !lastDown) selection = (selection + 1) % 3;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        // Adjustment
        double step = (selection == 0) ? 5.0 : (selection == 1) ? 0.005 : 0.1;
        if (gamepad1.right_bumper && !lastRB) modify(step);
        if (gamepad1.left_bumper && !lastLB) modify(-step);
        lastRB = gamepad1.right_bumper;
        lastLB = gamepad1.left_bumper;

        if (gamepad1.a) turret.resetEncoder();

        turret.setTargetAngle(targetAngle);
        turret.update();

        telemetry.addData("Selection", selection == 0 ? "Target Angle" : selection == 1 ? "kP" : "TicksPerDegree");
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("kP", TurretSubsystem.kP);
        telemetry.addData("Ticks Per Degree", TurretSubsystem.TICKS_PER_DEGREE);
        telemetry.addLine();
        telemetry.addData("Current Angle", "%.2f", turret.getCurrentAngle());
        telemetry.update();
    }

    private void modify(double step) {
        if (selection == 0) targetAngle += step;
        else if (selection == 1) TurretSubsystem.kP += step;
        else TurretSubsystem.TICKS_PER_DEGREE += step;
    }
}
