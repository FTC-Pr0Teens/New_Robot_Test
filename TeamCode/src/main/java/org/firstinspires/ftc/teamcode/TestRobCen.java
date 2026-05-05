package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp
public class TestRobCen extends OpMode {
    private MecanumDrive drive;
    private GamepadEx driverOp;

    private Hardware hw;

    @Override
    public void init() {
        hw = Hardware.getInstance(hardwareMap);

        drive = new MecanumDrive(hw.fL, hw.fR, hw.bL, hw.bR);
        driverOp = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        drive.driveRobotCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightY()
        );
    }

}