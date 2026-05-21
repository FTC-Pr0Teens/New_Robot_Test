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

/**
 * MAIN ROBOT OPMODE
 * 
 * CONTROLS:
 * 1. DRIVING:
 *    - Left Stick: Strafe/Move (Field Centric)
 *    - Right Stick X: Rotate
 * 
 * 2. MECHANISMS:
 *    - Button A: Intake ON/OFF (Toggle)
 *    - Right Bumper: Intake Direction (Toggle Forward/Reverse)
 *    - Button B: Manual Shooter Toggle (1500 RPM default)
 *    - Button Y: Sorting Mode Toggle (Enable Auto-Sort vs Bypass)
 *    - Button X: AUTO-SHOOT SEQUENCE (Spin up to 2300 RPM -> Sort -> Fire)
 * 
 * 3. UTILITIES:
 *    - Dpad Left: Reset Robot Heading (Yaw)
 *    - Button Back/Options: Reset Sorter Memory and Step
 *    - IF SORTER DISABLED (Bypass Mode):
 *      - Dpad Up: Manual Move to Slot 1
 *      - Dpad Right: Manual Move to Slot 2
 *      - Dpad Down: Manual Move to Slot 3
 */
@TeleOp(name="Main Robot OpMode")
public class mainOp extends OpMode {
    
    // Hardware and Subsystems
    private Hardware hw;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private IMU imu;
    private Sorter sorter;
    private ShooterSubsystem shooter;

    // Configuration
    private final double MANUAL_RPM = 1500.0;
    private final double AUTO_SHOOT_RPM = 2300.0;
    
    // Logic States
    private boolean intakeRunning = false;
    private boolean lastIntakeButton = false;
    private boolean intakeReversed = false;
    private boolean lastIntakeReverseButton = false;
    
    private boolean shooterRunning = false;
    private boolean lastBButton = false;

    private boolean sortingEnabled = true;
    private boolean lastYButton = false;

    private boolean lastXButton = false;
    private boolean autoShootActive = false;
    private int sortStep = 0;

    // Target sequence for sorting (Fires when X is pressed and speed is ready)
    private Sorter.BallColor[] targets = {
            Sorter.BallColor.GREEN,
            Sorter.BallColor.PURPLE,
            Sorter.BallColor.GREEN
    };

    @Override
    public void init() {
        // Initialize hardware bridge and subsystems
        hw = Hardware.getInstance(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap);

        // Initialize Mecanum Drive
        drive = new MecanumDrive(hw.fL, hw.fR, hw.bL, hw.bR);
        driverOp = new GamepadEx(gamepad1);

        // Initialize IMU for Field-Centric rotation tracking
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        // Initial Hardware Positioning
        if (sortingEnabled) {
            sorter.moveToSlot(0); 
            hw.flipper.setPosition(0.15); // Release/Ready position (LOW)
        } else {
            hw.flipper.setPosition(0.0); // Bypass position (UP)
        }

        // Set Sensor Gain for better color detection
        hw.ncs.setGain(2.0f);

        telemetry.speak("Ready to run");
    }

    @Override
    public void loop() {
        
        // --- 1. FIELD CENTRIC DRIVING ---
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        drive.driveFieldCentric(
                -driverOp.getLeftX(),
                -driverOp.getLeftY(),
                -driverOp.getRightX(),
                heading
        );

        // --- 2. INTAKE CONTROL (Button A Toggle & RB Direction Toggle) ---
        boolean currentIntakeButton = gamepad1.a;
        if (currentIntakeButton && !lastIntakeButton) {
            intakeRunning = !intakeRunning;
        }
        lastIntakeButton = currentIntakeButton;

        boolean currentIntakeReverseButton = gamepad1.right_bumper;
        if (currentIntakeReverseButton && !lastIntakeReverseButton) {
            intakeReversed = !intakeReversed;
        }
        lastIntakeReverseButton = currentIntakeReverseButton;
        
        // Apply power and manage flipper for outtake
        double intakePower = 0.0;
        if (intakeRunning) {
            intakePower = intakeReversed ? -0.8 : 0.8;
            
            // If we are reversing (outtaking), lift the flipper (0.0) to clear the path.
            // If we are going forward, the flipper position is handled by sortingEnabled logic below.
            if (intakeReversed) {
                hw.flipper.setPosition(0.0);
            } else if (sortingEnabled) {
                hw.flipper.setPosition(0.15);
            }
        }
        hw.intake.setPower(intakePower);

        // --- 3. MANUAL SHOOTER CONTROL (Button B Toggle) ---
        boolean currentBButton = gamepad1.b;
        if (currentBButton && !lastBButton) {
            autoShootActive = false; // Manual override cancels auto sequence
            shooterRunning = !shooterRunning;
            if (shooterRunning) {
                shooter.setTargetRPM(MANUAL_RPM);
                shooter.on();
            } else {
                shooter.off();
            }
        }
        lastBButton = currentBButton;

        // --- 4. AUTO-SHOOT SEQUENCE (Button X) ---
        boolean currentXButton = gamepad1.x;
        if (currentXButton && !lastXButton && sortingEnabled) {
            autoShootActive = true;
            shooterRunning = true;
            shooter.setTargetRPM(AUTO_SHOOT_RPM);
            shooter.on();
        }
        lastXButton = currentXButton;

        // Sequence logic: Wait for RPM -> Fire -> Repeat
        if (autoShootActive && sortingEnabled) {
            double currentRPM = shooter.getCurrentRPM();
            // Check if speed is within +/- 250 RPM of target (2300)
            if (Math.abs(currentRPM - AUTO_SHOOT_RPM) <= 400) {
                if (sortStep < targets.length) {
                    sorter.sortToColor(targets[sortStep]); // Move to the slot
                    sorter.transfer(shooter); // Release the ball (keeps flywheel updating)
                    sortStep++;
                } else {
                    // Sequence finished
                    autoShootActive = false;
                }
            }
        }

        // Keep shooter PID updating if active
        if (shooterRunning) {
            shooter.update();
        }

        // --- 5. SORTING MODE TOGGLE (Button Y Toggle) ---
        boolean currentYButton = gamepad1.y;
        if (currentYButton && !lastYButton) {
            sortingEnabled = !sortingEnabled;
            autoShootActive = false; // Disable auto-sequence if mode changes
            if (sortingEnabled) {
                sorter.moveToSlot(0);
                hw.flipper.setPosition(0.15);
            } else {
                hw.flipper.setPosition(0.0); // Bypass mode (UP)
            }
        }
        lastYButton = currentYButton;

        // --- 6. AUTOMATIC RECORDING & REMOVAL ---
        NormalizedRGBA colors = hw.ncs.getNormalizedColors();
        double currentHue = JavaUtil.colorToHue(colors.toColor());
        // Manage memory as balls enter/exit
        if (intakeRunning && sortingEnabled) {
            if (intakeReversed) {
                sorter.scanAndRemove(currentHue, colors.alpha);
            } else {
                sorter.scanAndRecord(currentHue, colors.alpha);
            }
        }

        // --- 7. UTILITIES & MANUAL SORTER ---
        // Reset Sorter System (Back Button)
        if (gamepad1.back) {
            sorter.reset();
            sortStep = 0;
            autoShootActive = false;
            if (sortingEnabled) {
                sorter.moveToSlot(0);
                hw.flipper.setPosition(0.15);
            }
        }
        
        // Manual Sorter Controls (Only in Bypass/Non-Sorting Mode)
        if (!sortingEnabled) {
            if (gamepad1.dpad_up)    sorter.moveToSlot(0);
            if (gamepad1.dpad_right) sorter.moveToSlot(1);
            if (gamepad1.dpad_down)  sorter.moveToSlot(2);
        }

        // Reset Gyro Heading (Dpad Left)
        if (gamepad1.dpad_left) {
            imu.resetYaw();
        }

        // --- 8. TELEMETRY DIAGNOSTICS ---
        telemetry.addData("Shooter", shooterRunning ? (autoShootActive ? "AUTO-SEQUENCE" : "MANUAL") : "OFF");
        telemetry.addData("RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("Intake", intakeRunning ? (intakeReversed ? "REVERSE" : "FORWARD") : "OFF");
        telemetry.addData("Sorter", sortingEnabled ? "ENABLED" : "BYPASS");
        telemetry.addData("Sort step", "%d / %d", sortStep, targets.length);
        telemetry.addData("Memory", Arrays.toString(sorter.getRecordedColors()));
        telemetry.addData("Hue", "%.1f", currentHue);
        telemetry.update();
    }
}
