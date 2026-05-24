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
import org.firstinspires.ftc.teamcode.hardware.HoodSubsystem;
import org.firstinspires.ftc.teamcode.hardware.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Sorter;
import org.firstinspires.ftc.teamcode.hardware.TurretSubsystem;
import org.firstinspires.ftc.teamcode.hardware.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;
import java.util.List;

/**
 * MAIN ROBOT OPMODE (Iterative)
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
    private TurretSubsystem turret;
    private HoodSubsystem hood;
    private VisionSubsystem vision;

    // Configuration
    private final double MANUAL_RPM = 1500.0;
    private final double AUTO_SHOOT_RPM = 2300.0;
    
    // Alliance Settings
    private enum Alliance { BLUE, RED, NONE }
    private Alliance currentAlliance = Alliance.NONE;
    
    // Target Tag IDs
    private final List<Integer> BLUE_TAGS = Arrays.asList(20);
    private final List<Integer> RED_TAGS = Arrays.asList(24);

    // Logic States
    private boolean intakeRunning = false;
    private boolean lastIntakeButton = false;
    private boolean intakeReversed = false;
    private boolean lastIntakeReverseButton = false;
    
    private boolean shooterRunning = false;
    private boolean lastBButton = false;

    private boolean autoSortingEnabled = true;
    private boolean lastYButton = false;

    private boolean turretLockEnabled = true;
    private boolean lastLB = false;

    private boolean lastXButton = false;
    private boolean autoShootActive = false;
    private int sortStep = 0;

    // Vision Smoothing
    private double smoothedBearing = 0;
    private final double VISION_SMOOTHING = 0.4; // Weight of new data (0.0 - 1.0)

    // Vision Calibration
    private int cameraGain = 25; // Lower default for bright environments
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // Target sequence for sorting
    private Sorter.BallColor[] targets = {
            Sorter.BallColor.GREEN,
            Sorter.BallColor.PURPLE,
            Sorter.BallColor.GREEN
    };

    @Override
    public void init() {
        hw = Hardware.getInstance(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        hood = new HoodSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);

        drive = new MecanumDrive(hw.fL, hw.fR, hw.bL, hw.bR);
        driverOp = new GamepadEx(gamepad1);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        // Initial Hardware Positions
        sorter.moveToSlot(0); 
        hw.flipper.setPosition(0.15); // LOW (Ready to receive)

        hw.ncs.setGain(2.0f);
        hood.setPosition(0.36); // Set to HOOD_MIN initially
    }

    @Override
    public void init_loop() {
        // Alliance Selection during Init
        if (gamepad1.x) currentAlliance = Alliance.BLUE;
        if (gamepad1.b) currentAlliance = Alliance.RED;

        // Camera Gain Calibration
        if (gamepad1.dpad_up && !lastDpadUp) cameraGain = Math.min(255, cameraGain + 5);
        if (gamepad1.dpad_down && !lastDpadDown) cameraGain = Math.max(0, cameraGain - 5);
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // Apply settings (Exposure remains at 6ms to prevent blur)
        vision.setManualExposure(6, cameraGain);

        telemetry.addData("Status", "READY - Select Alliance");
        telemetry.addData("Vision", vision.getCameraState());
        telemetry.addData("Camera Gain", cameraGain);
        telemetry.addData("Alliance", currentAlliance == Alliance.NONE ? "PRESS X (Blue) or B (Red)" : currentAlliance);
        telemetry.addLine("\nControls: X for BLUE, B for RED");
        telemetry.addLine("Calibrate Gain: Dpad UP/DOWN");
        telemetry.update();
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

        // --- 2. INTAKE CONTROL ---
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
        
        double intakePower = 0.0;
        if (intakeRunning && !sorter.isBusy()) {
            intakePower = intakeReversed ? -0.8 : 0.8;
            
            // Flipper Management
            if (intakeReversed) {
                hw.flipper.setPosition(0.0); // Lift for outtake
            } else {
                hw.flipper.setPosition(0.15); // Lower for holding/sorting
            }
        }
        hw.intake.setPower(intakePower);

        // --- 3. MANUAL SHOOTER CONTROL ---
        boolean currentBButton = gamepad1.b;
        if (currentBButton && !lastBButton) {
            autoShootActive = false;
            shooterRunning = !shooterRunning;
            if (shooterRunning) {
                shooter.setTargetRPM(MANUAL_RPM);
                shooter.on();
            } else {
                shooter.off();
            }
        }
        lastBButton = currentBButton;

        // --- 4. SHOOT SEQUENCE (AUTO or MANUAL) ---
        boolean currentXButton = gamepad1.x;
        if (currentXButton && !lastXButton) {
            autoShootActive = true;
            shooterRunning = true;
            shooter.setTargetRPM(AUTO_SHOOT_RPM);
            shooter.on();
        }
        lastXButton = currentXButton;

        if (autoShootActive) {
            if (!sorter.isBusy()) {
                double currentRPM = shooter.getCurrentRPM();
                // Sequence check: within +/- 250 RPM of target speed
                if (Math.abs(currentRPM - AUTO_SHOOT_RPM) <= 250) {
                    if (autoSortingEnabled) {
                        if (sortStep < targets.length) {
                            // Try to find the specific color
                            if (sorter.sortToColor(targets[sortStep])) {
                                sorter.startTransfer();
                                sortStep++;
                            } else {
                                // Specific color not found, skip to next in sequence
                                sortStep++;
                            }
                        } else {
                            // Finished the 3-ball target sequence
                            autoShootActive = false;
                            sortStep = 0;
                        }
                    } else {
                        // MANUAL MODE: Just fire from whatever slot you chose
                        sorter.startTransfer();
                        autoShootActive = false;
                    }
                }
            }
        }

        // --- 5. SUBSYSTEM UPDATES (NON-BLOCKING) ---
        shooter.update();
        sorter.update(shooter); // Sorter state machine handles the fire sequence
        
        // --- TURRET CONTROL ---
        // Toggle Target Lock with Left Bumper
        boolean currentLB = gamepad1.left_bumper;
        if (currentLB && !lastLB) {
            turretLockEnabled = !turretLockEnabled;
        }
        lastLB = currentLB;

        if (turretLockEnabled) {
            AprilTagDetection bestDetection = null;
            List<AprilTagDetection> detections = vision.getAllDetections();
            
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null && detection.ftcPose != null) {
                    boolean isAllianceTag = (currentAlliance == Alliance.BLUE && BLUE_TAGS.contains(detection.id)) ||
                                           (currentAlliance == Alliance.RED && RED_TAGS.contains(detection.id));
                    if (isAllianceTag) {
                        bestDetection = detection;
                        break;
                    }
                }
            }

            if (bestDetection != null) {
                // Low-pass Filter for bearing to stop jitter
                smoothedBearing = (bestDetection.ftcPose.bearing * VISION_SMOOTHING) + (smoothedBearing * (1.0 - VISION_SMOOTHING));
                turret.lockToTag(smoothedBearing);
                telemetry.addData("Turret Lock", "APRILTAG (ID %d)", bestDetection.id);
            } else {
                turret.lockToFieldAngle(0.0, heading);
                telemetry.addData("Turret Lock", "FIELD ANGLE (Searching...)");
            }
        } else {
            // Manual rotation with triggers
            double manualTurretPower = gamepad1.right_trigger - gamepad1.left_trigger;
            if (Math.abs(manualTurretPower) > 0.05) {
                turret.setManualPower(manualTurretPower * 0.4);
            } else {
                turret.setManualPower(0);
            }
            telemetry.addData("Turret Lock", "MANUAL");
        }
        turret.update();

        // --- 6. MODE TOGGLE ---
        boolean currentYButton = gamepad1.y;
        if (currentYButton && !lastYButton) {
            autoSortingEnabled = !autoSortingEnabled;
            hw.flipper.setPosition(0.15);
        }
        lastYButton = currentYButton;

        // --- 7. AUTOMATIC RECORDING & MANUAL SORTER ---
        if (!sorter.isBusy()) {
            NormalizedRGBA colors = hw.ncs.getNormalizedColors();
            double currentHue = JavaUtil.colorToHue(colors.toColor());
            
            if (autoSortingEnabled) {
                if (intakeRunning) {
                    if (intakeReversed) {
                        sorter.scanAndRemove(currentHue, colors.alpha);
                    } else {
                        sorter.scanAndRecord(currentHue, colors.alpha);
                    }
                }
            } else {
                if (gamepad1.dpad_up)    sorter.moveToSlot(0);
                if (gamepad1.dpad_right) sorter.moveToSlot(1);
                if (gamepad1.dpad_down)  sorter.moveToSlot(2);
            }
        }

        // --- 8. UTILITIES ---
        if (gamepad1.back) {
            sorter.reset();
            sortStep = 0;
            autoShootActive = false;
            sorter.moveToSlot(0);
            hw.flipper.setPosition(0.15);
        }
        
        if (gamepad1.dpad_left) {
            imu.resetYaw();
        }

        // --- 9. TELEMETRY ---
        telemetry.addData("Alliance", currentAlliance);
        telemetry.addData("Camera Gain", cameraGain);
        
        // --- AprilTag Vision Diagnostics ---
        List<AprilTagDetection> currentDetections = vision.getAllDetections();
        if (currentDetections != null && !currentDetections.isEmpty()) {
            telemetry.addData("Tags Visible", currentDetections.size());
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format(" > ID %d (%s) Brng: %.1f", 
                        detection.id, detection.metadata.name, detection.ftcPose.bearing));
                } else {
                    telemetry.addLine(String.format(" > ID %d (Unknown) - Move closer", detection.id));
                }
            }
        } else {
            telemetry.addData("Vision Status", "No Tags Seen");
        }

        telemetry.addData("Turret Angle", "%.1f", turret.getCurrentAngle());
        telemetry.addData("Recommended RPM", "%.0f", hood.getRecommendedRPM());
        telemetry.addData("Shooter", shooterRunning ? (autoShootActive ? "SEQ-ACTIVE" : "ON") : "OFF");
        telemetry.addData("RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("Sort Mode", autoSortingEnabled ? "AUTO" : "MANUAL (Dpads)");
        telemetry.addData("Sorter State", sorter.isBusy() ? "BUSY" : "READY");
        telemetry.addData("Memory", Arrays.toString(sorter.getRecordedColors()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (vision != null) {
            vision.close();
        }
    }
}
