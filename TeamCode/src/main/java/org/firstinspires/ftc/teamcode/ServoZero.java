package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp
public class ServoZero extends OpMode {
    private Servo sorter1;
    private Servo sorter2;
    private Servo sorter3;
    Hardware hw;
    @Override
    public void init() {
        hw = Hardware.getInstance(hardwareMap);

        sorter1 = hardwareMap.get(Servo.class,"sorter1" );
        sorter2 = hardwareMap.get(Servo.class,"sorter2" );
        sorter3 = hardwareMap.get(Servo.class,"sorter3" );

        sorter1.setPosition(0);
        sorter2.setPosition(0);
        sorter3.setPosition(0);
        hw.flipper.setPosition(0);

        telemetry.addData("Status", "Servos zeroed");
        telemetry.addData("Servo1", sorter1.getPosition());
        telemetry.addData("Servo2", sorter2.getPosition());
        telemetry.addData("Servo3", sorter3.getPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        hw.sorter1.setPosition(0);
        hw.sorter2.setPosition(0);
        hw.sorter3.setPosition(0);
        hw.flipper.setPosition(0);
    }
}