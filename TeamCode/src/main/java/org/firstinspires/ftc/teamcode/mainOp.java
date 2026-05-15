package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Sorter;

import java.util.Arrays;

@TeleOp
public class mainOp extends OpMode {
    Hardware hw;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private IMU imu;
    private Sorter sorter;

    private ShooterSubsystem shooter;
    private final double targetRPM = 1500.0;

    // Intake toggle (Gamepad A)
    boolean intakeRunning = false;
    boolean lastIntakeButton = false;

    // Shooter toggle (Gamepad B)
    boolean shooterRunning = false;
    boolean lastBButton = false;

    // Sorting Mode toggle (Gamepad Y)
    boolean sortingEnabled = true;
    boolean lastYButton = false;

    // Sorter sorting sequence (Gamepad X)
    boolean lastXButton = false;
    int sortStep = 0;

    Sorter.BallColor[] targets = {
            Sorter.BallColor.GREEN,
            Sorter.BallColor.GREEN,
            Sorter.BallColor.GREEN
    };

    @Override
    public void init() {
        hw = Hardware.getInstance(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap);

         drive = new MecanumDrive(hw.fL, hw.fR, hw.bL, hw.bR);
//        drive = new MecanumDrive(hw.bL, hw.bR, hw.fL, hw.fR);
        driverOp = new GamepadEx(gamepad1);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        if (sortingEnabled) {
            sorter.moveToSlot(0);
            hw.flipper.setDirection(Servo.Direction.FORWARD);
            hw.flipper.setPosition(0.15);
        } else {
            hw.flipper.setDirection(Servo.Direction.REVERSE);
            hw.flipper.setPosition(0.1);
        }

        telemetry.speak("Ready");
    }

    @Override
    public void loop() {
        // --- Driving ---
        // FIX #3: Use getRightX() for rotation, not getRightY()
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        drive.driveFieldCentric(driverOp.getLeftX(), driverOp.getLeftY(), -driverOp.getRightX(), heading);

        // --- Intake toggle (gamepad1 a) ---
        boolean currentIntakeButton = gamepad1.a;
        if (currentIntakeButton && !lastIntakeButton)
            intakeRunning = !intakeRunning;
        lastIntakeButton = currentIntakeButton;

        hw.intake.set(intakeRunning ? 0.8 : 0.0);

        // --- Shooter toggle (gamepad1 b) ---
        // FIX #1 & #2: Only call on()/off() on button press, not every loop.
        // setTargetRPM is only called when turning on.
        boolean currentBButton = gamepad1.b;
        if (currentBButton && !lastBButton) {
            shooterRunning = !shooterRunning;
            if (shooterRunning) {
                shooter.setTargetRPM(targetRPM);
                shooter.on();
            } else {
                shooter.off();
            }
        }
        lastBButton = currentBButton;

        // FIX #1: Only call update() when running; no else branch that calls toggle()
        if (shooterRunning) {
            shooter.update();
        }

        // --- Sorting Mode Toggle (gamepad1 y) ---
        boolean currentYButton = gamepad1.y;
        if (currentYButton && !lastYButton) {
            sortingEnabled = !sortingEnabled;
            if (sortingEnabled) {
                sorter.moveToSlot(0);
                hw.flipper.setDirection(Servo.Direction.FORWARD);
                hw.flipper.setPosition(0.15);
            } else {
                hw.flipper.setDirection(Servo.Direction.FORWARD);
                // REVERSE
                hw.flipper.setPosition(0);
                //0.1
            }
        }
        lastYButton = currentYButton;

        // --- Recording Phase ---
        if (intakeRunning && sortingEnabled) {
            NormalizedRGBA colors = hw.ncs.getNormalizedColors();
            double hue = JavaUtil.colorToHue(colors.toColor());
            sorter.scanAndRecord(hue);
        }

        // --- Sorting Phase (gamepad1 x) ---
        boolean currentXButton = gamepad1.x;
        if (currentXButton && !lastXButton && sortingEnabled) {
            if (sortStep < targets.length) {
                sorter.sortToColor(targets[sortStep]);
                sorter.transfer();
                sortStep++;
            }
        }
        lastXButton = currentXButton;

        // --- Telemetry ---
        telemetry.addData("Shooter", shooterRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("Sorting Mode", sortingEnabled ? "ENABLED (Auto)" : "DISABLED (Flipper UP)");
        telemetry.addData("Recorded List", Arrays.toString(sorter.getRecordedColors()));
        telemetry.addData("Robot Heading", heading);
        telemetry.update();
    }
}