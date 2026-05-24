package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Hardware {

    //singleton
    private static Hardware instance;

    // Motors
    public final Motor fL;
    public final Motor fR;
    public final Motor bL;
    public final Motor bR;
    public final DcMotor intake;
    public final DcMotorEx shooter;
    public final DcMotorEx turret;
    public final Servo sorter1;
    public final Servo sorter2;
    public final Servo flipper;
    public final Servo hood;

    public final NormalizedColorSensor ncs;
    public final DistanceSensor ds;



    public Hardware(HardwareMap hwMap) {
        this.fL = new Motor(hwMap, "fl");//front left
        this.fR = new Motor(hwMap, "fr");//front right
        this.bL = new Motor(hwMap, "bl");//back left
        this.bR = new Motor(hwMap, "br");//back right
        this.intake = hwMap.get(DcMotor.class, "intake");//intake
        this.shooter = hwMap.get(DcMotorEx.class, "shooter");
        this.turret = hwMap.get(DcMotorEx.class, "turret");

        this.intake.setDirection(DcMotor.Direction.FORWARD);
        this.shooter.setDirection(DcMotor.Direction.FORWARD);
        this.turret.setDirection(DcMotor.Direction.FORWARD);

        this.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.sorter1 = hwMap.get(Servo.class, "sorter1");
        this.sorter2 = hwMap.get(Servo.class, "sorter2");
        this.flipper = hwMap.get(Servo.class, "flipper");
        this.hood = hwMap.get(Servo.class, "hood");

        this.flipper.setDirection(Servo.Direction.FORWARD);
        this.sorter1.setDirection(Servo.Direction.FORWARD);
        this.sorter2.setDirection(Servo.Direction.REVERSE);


        ncs = hwMap.get(NormalizedColorSensor.class, "color1");
        ds = hwMap.get(DistanceSensor.class, "color1");

    }

    public static Hardware getInstance(HardwareMap hwMap) {
        if (instance == null) {
            instance = new Hardware(hwMap);
        }
        return instance;
    }


}

