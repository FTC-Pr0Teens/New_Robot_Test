package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class Hardware {

    private static Hardware instance;

    // Motors
    public final Motor fL, fR, bL, bR;
    public final DcMotor intake;
    public final DcMotorEx shooter, turret;
    public final Servo sorter1, sorter2, flipper, hood;

    public final NormalizedColorSensor ncs;
    public final DistanceSensor ds;
    
    // Pinpoint & IMU
    public final GoBildaPinpointDriver pinpoint;
    public final IMU imu;

    public Hardware(HardwareMap hwMap) {
        this.fL = new Motor(hwMap, "fl");
        this.fR = new Motor(hwMap, "fr");
        this.bL = new Motor(hwMap, "bl");
        this.bR = new Motor(hwMap, "br");
        
        this.intake = hwMap.get(DcMotor.class, "intake");
        this.shooter = hwMap.get(DcMotorEx.class, "shooter");
        this.turret = hwMap.get(DcMotorEx.class, "turret");

        this.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.sorter1 = hwMap.get(Servo.class, "sorter1");
        this.sorter2 = hwMap.get(Servo.class, "sorter2");
        this.flipper = hwMap.get(Servo.class, "flipper");
        this.hood = hwMap.get(Servo.class, "hood");

        this.ncs = hwMap.get(NormalizedColorSensor.class, "color1");
        this.ds = hwMap.get(DistanceSensor.class, "color1");
        
        this.pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        this.imu = hwMap.get(IMU.class, "imu");
        
        // Initialize Hub IMU
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        this.imu.initialize(parameters);
    }

    public static Hardware getInstance(HardwareMap hwMap) {
        if (instance == null) instance = new Hardware(hwMap);
        return instance;
    }
}
