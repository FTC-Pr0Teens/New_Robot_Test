package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * Bare goBILDA 6000 RPM motor held at 3000 RPM. No tuning, nothing to set up.
 *
 * It starts spinning the moment you press PLAY. B stops it, A starts it again.
 *
 * The coefficients below are not magic numbers someone guessed, they are the
 * standard recipe: F comes from the motor's rated speed, P is a tenth of F,
 * I is a tenth of P. They are good enough here because 3000 RPM is only half
 * of what this motor can do, so the controller has plenty of spare power to
 * correct with. Ask for 5500 and you would have to tune properly.
 */
@TeleOp(name = "Flywheel Test (3000 RPM)", group = "Tests")
public class FlywheelTest extends LinearOpMode {

    /** Must match the name in the Robot Controller configuration exactly. */
    private static final String MOTOR_NAME = "flywheel";

    private static final double TICKS_PER_REV = 28.0;   // bare Yellow Jacket
    private static final double TARGET_RPM = 6000.0;

    // 3000 RPM = 50 revs per second = 1400 encoder ticks per second.
    private static final double TARGET_TPS = TARGET_RPM / 60.0 * TICKS_PER_REV;

    // 32767 is the hub's full scale output. Divided by the motor's top speed
    // in ticks per second (6000 RPM = 2800 tps), that is the feedforward.
    private static final double F = 32767.0 / 2800.0;   // 11.70
    private static final double P = 0.1 * F;            // 1.17
    private static final double I = 0.1 * P;            // 0.117
    private static final double D = 0.0;

    @Override
    public void runOpMode() {
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // Let it coast down when stopped instead of the motor fighting the wheel.
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // The SDK caps most motors at 85 percent of rated speed by default.
        // Undo that so the full range is actually available.
        MotorConfigurationType type = flywheel.getMotorType().clone();
        type.setAchieveableMaxRPMFraction(1.0);
        flywheel.setMotorType(type);

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Order matters. Setting a run mode wipes the coefficients back to the
        // motor's defaults, so these have to come after setMode, not before.
        flywheel.setVelocityPIDFCoefficients(P, I, D, F);

        telemetry.addLine("Ready. PLAY starts it at 3000 RPM.");
        telemetry.update();

        waitForStart();

        boolean running = true;      // spins as soon as you press play
        boolean lastA = false, lastB = false;

        while (opModeIsActive()) {
            if (gamepad1.a && !lastA) running = true;
            if (gamepad1.b && !lastB) running = false;
            lastA = gamepad1.a;
            lastB = gamepad1.b;

            if (running) {
                flywheel.setVelocity(TARGET_TPS);
            } else {
                flywheel.setPower(0);
            }

            double rpm = flywheel.getVelocity() / TICKS_PER_REV * 60.0;

            telemetry.addData("state", running ? "RUNNING (B stops)" : "STOPPED (A starts)");
            telemetry.addData("target", "%.0f RPM", TARGET_RPM);
            telemetry.addData("actual", "%.0f RPM", rpm);
            telemetry.addData("at speed", running && Math.abs(TARGET_RPM - rpm) < 100);
            telemetry.update();
        }

        flywheel.setPower(0);
    }
}