package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
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
    public final Motor intake;
    public final MotorEx shooter;
    public final Servo sorter1;
    public final Servo sorter2;
    public final Servo flipper;

    public final NormalizedColorSensor ncs;
    public final DistanceSensor ds;



    public Hardware(HardwareMap hwMap) {
        this.fL = new Motor(hwMap, "fl");//front left
        this.fR = new Motor(hwMap, "fr");//front right
        this.bL = new Motor(hwMap, "bl");//back left
        this.bR = new Motor(hwMap, "br");//back right
        this.intake = new Motor(hwMap,"intake");//intake
        this.shooter = new MotorEx(hwMap, "shooter", Motor.GoBILDA.BARE);


        fL.setInverted(false);
        fR.setInverted(false);
        bL.setInverted(false);
        bR.setInverted(false);

        this.intake.setInverted(false);

        this.sorter1 = hwMap.get(Servo.class, "sorter1");
        this.sorter2 = hwMap.get(Servo.class, "sorter2");
        this.flipper = hwMap.get(Servo.class, "flipper");

        this.flipper.setDirection(Servo.Direction.REVERSE);
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

