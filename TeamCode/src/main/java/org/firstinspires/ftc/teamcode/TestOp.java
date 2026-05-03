package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@TeleOp
public class TestOp extends OpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive drive;
    private GamepadEx driverOp;

    @Override
    public void init() {
        /* instantiate motors */
        Motor fL = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
        Motor fR = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
        Motor bL = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
        Motor bR = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);

        drive = new MecanumDrive(fL, fR, bL, bR);
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