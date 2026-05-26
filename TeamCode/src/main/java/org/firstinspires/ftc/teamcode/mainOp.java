package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
 * MAIN ROBOT OPMODE (Iterative)
 */
@TeleOp(name="Main Robot OpMode")
public class mainOp extends OpMode {
    
    private Hardware hw;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private IMU imu;
    private Sorter sorter;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private Follower follower;

    private final double MANUAL_RPM = 1500.0;
    private final double AUTO_SHOOT_RPM = 3200.0;
    
    private enum Alliance { BLUE, RED, NONE }
    private Alliance currentAlliance = Alliance.NONE;
    
    private final List<Integer> BLUE_TAGS = Arrays.asList(20);
    private final List<Integer> RED_TAGS = Arrays.asList(24);

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

    private int cameraGain = 25; 
    
    // Vision Preview Toggle
    // TODO: DISABLE PREVIEW FOR COMPETITION TO IMPROVE PERFORMANCE
    private boolean previewEnabled = true;
    private boolean lastStartButton = false;

    private int sequenceIndex = 1; // Default: PGP
    private Sorter.BallColor[][] possibleSequences = {
        {Sorter.BallColor.GREEN, Sorter.BallColor.PURPLE, Sorter.BallColor.PURPLE},
        {Sorter.BallColor.PURPLE, Sorter.BallColor.GREEN, Sorter.BallColor.PURPLE},
        {Sorter.BallColor.PURPLE, Sorter.BallColor.PURPLE, Sorter.BallColor.GREEN}
    };
    private String[] sequenceNames = {"G-P-P", "P-G-P", "P-P-G"};
    private Sorter.BallColor[] targets;

    private double smoothedBearing = 0;
    private final double VISION_SMOOTHING = 0.15;
    private ElapsedTime tagLostTimer = new ElapsedTime();

    @Override
    public void init() {
        hw = Hardware.getInstance(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);

        drive = new MecanumDrive(hw.fL, hw.fR, hw.bL, hw.bR);
        driverOp = new GamepadEx(gamepad1);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

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
    }

    @Override
    public void init_loop() {
        // Alliance Selection
        if (gamepad1.x) currentAlliance = Alliance.BLUE;
        if (gamepad1.b) currentAlliance = Alliance.RED;

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
    public void loop() {
        follower.update();
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        drive.driveFieldCentric(-driverOp.getLeftX(), -driverOp.getLeftY(), -driverOp.getRightX(), heading);

        // --- CAMERA PREVIEW TOGGLE (Start Button) ---
        boolean currentStart = gamepad1.start;
        if (currentStart && !lastStartButton) {
            previewEnabled = !previewEnabled;
            vision.setPreviewEnabled(previewEnabled);
        }
        lastStartButton = currentStart;

        // --- INTAKE & OUTTAKE ---
        boolean currentIntake = gamepad1.a;
        if (currentIntake && !lastIntakeButton) intakeRunning = !intakeRunning;
        lastIntakeButton = currentIntake;

        boolean currentRev = gamepad1.right_bumper;
        if (currentRev && !lastIntakeReverseButton) intakeReversed = !intakeReversed;
        lastIntakeReverseButton = currentRev;
        
        double intakePower = 0;
        if (intakeRunning && !sorter.isBusy()) {
            intakePower = intakeReversed ? -0.8 : 0.8;
            hw.flipper.setPosition(intakeReversed ? 0.0 : 0.15);
        }
        if (!sorter.isBusy()) {
            hw.intake.setPower(intakePower);
        }

        // --- SHOOT SEQUENCE ---
        if (gamepad1.b && !lastBButton) {
            autoShootActive = false;
            shooterRunning = !shooterRunning;
            if (shooterRunning) shooter.setTargetRPM(MANUAL_RPM);
            else shooter.off();
        }
        lastBButton = gamepad1.b;

        if (gamepad1.x && !lastXButton) {
            autoShootActive = true;
            shooterRunning = true;
            shooter.setTargetRPM(AUTO_SHOOT_RPM);
            shooter.on();
        }
        lastXButton = gamepad1.x;

        if (autoShootActive && !sorter.isBusy()) {
            if (Math.abs(shooter.getCurrentRPM() - AUTO_SHOOT_RPM) <= 100) {
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

        // --- TURRET & VISION ---
        boolean currentLB = gamepad1.left_bumper;
        if (currentLB && !lastLB) {
            turretLockEnabled = !turretLockEnabled;
            // Ensure vision processing is active if locking is enabled
            if (turretLockEnabled) {
                previewEnabled = true;
                vision.setPreviewEnabled(true);
            }
        }
        lastLB = currentLB;

        List<AprilTagDetection> detections = vision.getAllDetections();
        AprilTagDetection best = null;
        for (AprilTagDetection d : detections) {
            if (d.metadata != null && d.ftcPose != null) {
                if ((currentAlliance == Alliance.BLUE && BLUE_TAGS.contains(d.id)) ||
                    (currentAlliance == Alliance.RED && RED_TAGS.contains(d.id))) {
                    best = d; break;
                }
            }
        }

        if (turretLockEnabled) {
            if (best != null) {
                smoothedBearing = (best.ftcPose.bearing * VISION_SMOOTHING) + (smoothedBearing * (1.0 - VISION_SMOOTHING));
                turret.lockToTag(smoothedBearing);
                tagLostTimer.reset();
            } else if (tagLostTimer.seconds() > 0.5) {
                // Only snap to field angle 0 if tag is lost for > 0.5s
                turret.lockToFieldAngle(0.0, heading);
            }
            // else: hold last targetAngle while searching for tag
        } else {
            double p = gamepad1.right_trigger - gamepad1.left_trigger;
            if (Math.abs(p) > 0.05) turret.setManualPower(p * 0.4);
            else turret.setManualPower(0);
        }
        turret.update();

        // --- UTILS ---
        if (gamepad1.y && !lastYButton) autoSortingEnabled = !autoSortingEnabled;
        lastYButton = gamepad1.y;

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

        // --- 9. TELEMETRY ---
        telemetry.addData("Preview", previewEnabled ? "ON" : "OFF (Start Button to toggle)");
        telemetry.addData("Alliance", currentAlliance);
        
        if (detections.isEmpty()) {
            telemetry.addLine("Vision: No tags seen");
        } else {
            for (AprilTagDetection d : detections) {
                telemetry.addLine(String.format(Locale.US, "ID %d: Bearing %.1f", d.id, d.ftcPose != null ? d.ftcPose.bearing : 0));
            }
        }

        telemetry.addData("RPM", "%.0f", shooter.getCurrentRPM());
        telemetry.addData("Memory", Arrays.toString(sorter.getRecordedColors()));
        
        Pose robotPose = follower.getPose();
        telemetry.addData("Pose X", "%.2f", robotPose.getX());
        telemetry.addData("Pose Y", "%.2f", robotPose.getY());
        telemetry.addData("Pose Heading", "%.2f°", Math.toDegrees(robotPose.getHeading()));

        telemetry.update();
    }

    @Override
    public void stop() { if (vision != null) vision.close(); }
}
