package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Sorter;
import org.firstinspires.ftc.teamcode.hardware.TurretSubsystem;
import org.firstinspires.ftc.teamcode.hardware.VisionSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * MAIN ROBOT OPMODE (Iterative) - TeleOp Only
 */
@Configurable
@TeleOp(name="Main Robot OpMode")
public class mainOp extends OpMode {
    
    private Hardware hw;
    private GamepadEx driverOp;
    private IMU imu;
    private Sorter sorter;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private Follower follower;

    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private boolean isRobotCentric = false; // Field Centric by default

    public static Pose startingPose = new Pose(0, 0, Math.toRadians(90));

    private final double MANUAL_RPM = 1500.0;
    
    private enum Alliance { BLUE, RED, NONE }
    private Alliance currentAlliance = Alliance.NONE;
    
    private boolean intakeRunning = false;
    private boolean intakeReversed = false;
    private boolean shooterRunning = false;
    private boolean autoSortingEnabled = true;
    private boolean turretLockEnabled = true;
    private boolean autoShootActive = false;
    private int sortStep = 0;

    private double manualHoodPos = 0.5;
    private boolean manualHoodEnabled = false;

    private int cameraGain = 25; 
    private boolean previewEnabled = true;

    // Edge Detection States
    private boolean lastA = false, lastB = false, lastX = false, lastY = false, last2B = false, last2X = false;
    private boolean lastLB = false, lastRB = false, lastStart = false, lastRSB = false;

    private int sequenceIndex = 1; // Default: PGP
    private Sorter.BallColor[][] possibleSequences = {
        {Sorter.BallColor.GREEN, Sorter.BallColor.PURPLE, Sorter.BallColor.PURPLE},
        {Sorter.BallColor.PURPLE, Sorter.BallColor.GREEN, Sorter.BallColor.PURPLE},
        {Sorter.BallColor.PURPLE, Sorter.BallColor.PURPLE, Sorter.BallColor.GREEN}
    };
    private String[] sequenceNames = {"G-P-P", "P-G-P", "P-P-G"};
    private Sorter.BallColor[] targets;

    @Override
    public void init() {
        hw = Hardware.getInstance(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap);

        driverOp = new GamepadEx(gamepad1);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        turret = new TurretSubsystem(hardwareMap);
        turret.setFollower(follower);
        turret.setGoalPosition(10, 140); // Default to Blue goal
        vision = new VisionSubsystem(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        sorter.moveToSlot(0); 
        hw.flipper.setPosition(0.15);
        hw.ncs.setGain(2.0f);
        targets = possibleSequences[sequenceIndex];

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init_loop() {
        // Alliance Selection & Goal Setting (Inches)
        if (gamepad1.x) {
            currentAlliance = Alliance.BLUE;
            turret.setGoalPosition(0, 144);
        }
        if (gamepad1.b) {
            currentAlliance = Alliance.RED;
            turret.setGoalPosition(144, 144);
        }

        // Sequence Selection
        if (gamepad1.dpad_left)  sequenceIndex = 0;
        if (gamepad1.dpad_up)    sequenceIndex = 1;
        if (gamepad1.dpad_right) sequenceIndex = 2;
        targets = possibleSequences[sequenceIndex];

        // Camera Gain Tuning
        if (gamepad1.left_trigger > 0.5) cameraGain = Math.max(0, cameraGain - 1);
        if (gamepad1.right_trigger > 0.5) cameraGain = Math.min(255, cameraGain + 1);
        vision.setManualExposure(6, cameraGain);

        // Turret Zeroing (A Button during Init)
        if (gamepad1.a) {
            turret.resetEncoder();
        }

        telemetry.addData("Alliance", currentAlliance == Alliance.NONE ? "X (Blue) / B (Red)" : currentAlliance);
        telemetry.addData("Sequence", sequenceNames[sequenceIndex]);
        telemetry.addData("Gain", cameraGain);
        telemetry.addLine("\nA: ZERO TURRET");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        Pose currentPose = follower.getPose();
        if (currentPose == null) {
            telemetry.addLine("Odometry Not Init...");
            telemetry.update();
            return;
        }

        // --- DRIVE CONTROL ---
        double forward = gamepad1.left_stick_x;
        double strafe = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

        if (gamepad1.right_stick_button && !lastRSB) slowMode = !slowMode;
        lastRSB = gamepad1.right_stick_button;

        if (slowMode) {
            forward *= slowModeMultiplier;
            strafe *= slowModeMultiplier;
            turn *= slowModeMultiplier;
        }

        if (gamepad1.dpad_up) isRobotCentric = true;
        if (gamepad1.dpad_down) isRobotCentric = false;

        follower.setTeleOpDrive(forward, strafe, turn, isRobotCentric);

        // --- CAMERA PREVIEW ---
        if (gamepad1.start && !lastStart) {
            previewEnabled = !previewEnabled;
            vision.setPreviewEnabled(previewEnabled);
        }
        lastStart = gamepad1.start;

        // --- INTAKE ---
        if (gamepad1.a && !lastA) intakeRunning = !intakeRunning;
        lastA = gamepad1.a;

        if (gamepad1.right_bumper && !lastRB) intakeReversed = !intakeReversed;
        lastRB = gamepad1.right_bumper;
        
        if (!sorter.isBusy()) {
            if (intakeRunning) {
                hw.intake.setPower(intakeReversed ? -0.8 : 0.8);
                hw.flipper.setPosition(intakeReversed ? 0.0 : 0.15);
            } else {
                hw.intake.setPower(0);
            }
        }

        // --- SHOOT ---
        if (gamepad1.b && !lastB) {
            autoShootActive = false;
            shooterRunning = !shooterRunning;
            if (shooterRunning) shooter.setTargetRPM(MANUAL_RPM);
            else shooter.off();
        }
        lastB = gamepad1.b;

        if (gamepad1.x && !lastX) {
            autoShootActive = true;
            shooterRunning = true;
            shooter.setTargetRPM(turret.getShootRPM());
            shooter.on();
        }
        lastX = gamepad1.x;

        if (autoShootActive && !sorter.isBusy()) {
            double dynamicTarget = turret.getShootRPM();
            shooter.setTargetRPM(dynamicTarget);

            if (Math.abs(shooter.getCurrentRPM() - dynamicTarget) <= 150) {
                if (autoSortingEnabled) {
                    if (sortStep < targets.length) {
                        if (sorter.sortToColor(targets[sortStep])) sorter.startTransfer();
                        sortStep++;
                    } else {
                        autoShootActive = false;
                        sortStep = 0;
                    }
                } else {
                    sorter.startTransfer();
                    autoShootActive = false;
                }
            }
        }

        shooter.update();
        sorter.update(shooter); 

        // --- TURRET ---
        if (gamepad1.left_bumper && !lastLB) {
            turretLockEnabled = !turretLockEnabled;
            if (turretLockEnabled) {
                previewEnabled = true;
                vision.setPreviewEnabled(true);
            }
        }
        lastLB = gamepad1.left_bumper;

        List<AprilTagDetection> detections = vision.getAllDetections();
        if (turretLockEnabled) {
            turret.update();
        } else {
            double p = gamepad1.right_trigger - gamepad1.left_trigger;
            if (Math.abs(p) > 0.05) turret.setManualPower(p * 0.4);
            else turret.setManualPower(0);
        }

        // --- UTILS ---
        if (gamepad1.y && !lastY) autoSortingEnabled = !autoSortingEnabled;
        lastY = gamepad1.y;

        NormalizedRGBA c = hw.ncs.getNormalizedColors();
        double h = JavaUtil.colorToHue(c.toColor());
        if (!sorter.isBusy()) {
            if (autoSortingEnabled && intakeRunning) {
                if (intakeReversed) sorter.scanAndRemove(c.alpha);
                else sorter.scanAndRecord(h, c.alpha);
            } else if (!autoSortingEnabled) {
                if (gamepad1.dpad_up) sorter.moveToSlot(0);
                if (gamepad1.dpad_right) sorter.moveToSlot(1);
                if (gamepad1.dpad_down) sorter.moveToSlot(2);
            }
        }
        if (gamepad1.dpad_left) imu.resetYaw();

        // --- HOOD CONTROL (Gamepad 2) ---
        if (Math.abs(gamepad2.left_stick_y) > 0.05) {
            manualHoodEnabled = true;
            manualHoodPos = Range.clip(manualHoodPos - gamepad2.left_stick_y * 0.005, 0, 1);
            turret.setManualHood(manualHoodPos);
        }
        if (gamepad2.y) {
            manualHoodEnabled = false;
            turret.disableManualHood();
        }

        // --- FLYWHEEL CONTROL (Gamepad 2) ---
        if (gamepad2.b && !last2B) {
            shooter.setTargetRPM(shooter.getCurrentRPM()+ 200);
            shooter.on();
        }
        last2B = gamepad2.b;
        if (gamepad2.x && !last2X) {
            shooter.setTargetRPM(turret.getShootRPM()-200);
            shooter.on();
        }
        last2X = gamepad2.x;

        // --- TELEMETRY ---
        telemetryM.debug("position", currentPose);
        
        telemetry.addLine("--- TURRET DEBUG ---");
        telemetry.addData("Lock Enabled", turretLockEnabled);
        telemetry.addData("Alliance", currentAlliance);
        telemetry.addData("Goal Field Angle", "%.2f°", turret.getTargetFieldAngle());
        telemetry.addData("Robot Heading", "%.2f°", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Distance to Goal", "%.2f in", turret.getDistance());
        telemetry.addData("Current Angle", "%.2f", turret.getCurrentAngle());
        telemetry.addData("Target Angle", "%.2f", turret.getTargetAngle());
        telemetry.addData("Error", "%.2f", turret.getError());
        telemetry.addData("Motor Power", "%.2f", turret.getRequestedPower());
        telemetry.addData("Target RPM", "%.0f", turret.getShootRPM());
        telemetry.addData("Hood Mode", manualHoodEnabled ? "MANUAL: " + String.format(Locale.US, "%.3f", manualHoodPos) : "AUTO");
        
        telemetry.addLine("\n--- STATUS ---");
        telemetry.addData("Drive Mode", isRobotCentric ? "ROBOT CENTRIC" : "FIELD CENTRIC");
        telemetry.addData("RPM", shooter.getCurrentRPM());
        telemetry.addData("Memory", Arrays.toString(sorter.getRecordedColors()));

        if (detections == null || detections.isEmpty()) {
            telemetry.addLine("Vision: No tags seen");
        } else {
            for (AprilTagDetection d : detections) {
                telemetry.addLine(String.format(Locale.US, "ID %d: Bearing %.1f", d.id, d.ftcPose != null ? d.ftcPose.bearing : 0));
            }
        }
        telemetry.update();
    }

    @Override
    public void stop() { if (vision != null) vision.close(); }
}
