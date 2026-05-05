package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Sorter;

import java.util.Arrays;

@TeleOp
public class mainOp extends OpMode {
    Hardware hw;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private IMU imu;
    private Sorter sorter;

    // Intake toggle
    boolean intakeRunning = false;
    boolean lastIntakeButton = false;

    // Sorter sorting sequence
    boolean lastXButton = false;
    int sortStep = 0;
    
    // The sequence of colors we want to score
    Sorter.BallColor[] targets = {
            Sorter.BallColor.GREEN,
            Sorter.BallColor.GREEN,
            Sorter.BallColor.GREEN
    };

    @Override
    public void init() {
        hw = Hardware.getInstance(hardwareMap);
        sorter = new Sorter(hardwareMap, telemetry);

        drive = new MecanumDrive(hw.fL, hw.fR, hw.bL, hw.bR);
        driverOp = new GamepadEx(gamepad1);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        // --- Driving ---
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        drive.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightY(),
                heading);

        // --- Intake toggle (gamepad1 a) ---
        boolean currentIntakeButton = gamepad1.a;
        if (currentIntakeButton && !lastIntakeButton)
            intakeRunning = !intakeRunning;

        hw.intake.set(intakeRunning ? 0.8 : 0.0);

        // --- Recording Phase ---
        // Only scan if intake is running AND we haven't filled all slots
        if (intakeRunning) {
            NormalizedRGBA colors = hw.ncs.getNormalizedColors();
            double hue = JavaUtil.colorToHue(colors.toColor());
            sorter.scanAndRecord(hue);
        }

        lastIntakeButton = currentIntakeButton;

        // --- Sorting Phase (gamepad1 x) ---
        // Changed to ONE SORT PER BUTTON PRESS for better control and debugging
        boolean currentXButton = gamepad1.x;
        if (currentXButton && !lastXButton) {
            if (sortStep < targets.length) {
                sorter.sortToColor(targets[sortStep]);
                sorter.transfer();
                sortStep++;
            }
        }
        lastXButton = currentXButton;

        // --- Reset (gamepad1 b) ---
        if (gamepad1.b) {
            sorter.reset();
            sortStep = 0;
        }

        // --- Telemetry ---
        telemetry.addData("Intake State", intakeRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Next Target", sortStep < targets.length ? targets[sortStep] : "DONE");
        telemetry.addData("Sort Step", sortStep + " of " + targets.length);
        telemetry.addData("Recorded List", Arrays.toString(sorter.getRecordedColors()));

        telemetry.update();
    }
}
