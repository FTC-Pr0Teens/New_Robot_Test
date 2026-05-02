package org.firstinspires.ftc.teamcode.mechanisms;

public class SorterControl{
    private Servo s1;
    private Servo s2;
    private Servo s3;

    private NormalizedColorSensor cs;

    public enum DetectedColor{
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public void init(hardwareMap hw){
        this.hw = hw;

        s1 = hw.get(Servo.class, "sorter1");
        s2 = hw.get(Servo.class, "sorter2");
        s3 = hw.get(Servo.class, "sorter3");
        cs = hw.get(NormalizedColorSensor, "colorsensor1");

        s1.setPosition(0.0);
        s2.setPosition(0.0);
        s3.setPosition(0.0);
        
    }

    public DetectedColor getDetectedColor(Telemetry tl){
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // Returns RGBA Values between 0 - 1

        float normRed, normGreen, normPurple;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        tl.addData("Red" , normRed);
        tl.addData("Blue", normBlue);
        tl.addData("Green", normGreen);
        
        // Calibirate and add later
        
        return DetectedColor.UNKNOWN;
    }

}