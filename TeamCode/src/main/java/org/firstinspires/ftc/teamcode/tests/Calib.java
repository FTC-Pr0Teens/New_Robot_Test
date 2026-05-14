package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp
public class Calib extends OpMode {
    Hardware hw;
    @Override
    public void init() {
        hw = Hardware.getInstance(hardwareMap);
    }

    @Override
    public void loop() {
        //Using MM to calculate distance from sensor to ball
       double distance = hw.ds.getDistance(DistanceUnit.MM);
       telemetry.addData("Distance" , distance);

    }
}
