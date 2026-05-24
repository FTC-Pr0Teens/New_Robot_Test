package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class HoodSubsystem {
    private final Hardware hw;

    // --- TUNABLE CONSTANTS (From Tutorial) ---
    private final double CAMERA_HEIGHT = 0.35; // meters
    private final double CAMERA_ANGLE = Math.toRadians(25); // Radians
    private final double TARGET_HEIGHT = 1.05; // meters
    
    private final double HOOD_MIN = 0.36;
    private final double HOOD_MAX = 0.75;
    
    private final double MIN_DISTANCE = 0.3; // meters
    private final double MAX_DISTANCE = 3.0; // meters

    private final double MIN_RPM = 1500;
    private final double MAX_RPM = 2800;

    private double currentDistance = 0.0;
    private double recommendedRPM = 2300;

    public HoodSubsystem(HardwareMap hwMap) {
        this.hw = Hardware.getInstance(hwMap);
    }

    /**
     * Updates hood position based on target elevation angle.
     * @param elevationDeg Elevation angle from camera to tag in degrees (ftcPose.elevation)
     */
    public void update(double elevationDeg) {
        // Calculate Distance: (h2-h1) / tan(a1 + a2)
        double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(CAMERA_ANGLE + Math.toRadians(elevationDeg));
        distance *= 0.85; // Correction factor from tutorial
        
        this.currentDistance = Range.clip(distance, MIN_DISTANCE, MAX_DISTANCE);

        // Normalize distance for interpolation
        double normalized = (currentDistance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
        normalized = Range.clip(normalized, 0, 1);

        // Apply cubic curve for hood position
        double hoodCurve = Math.pow(normalized, 3.0);
        double hoodPos = HOOD_MIN + hoodCurve * (HOOD_MAX - HOOD_MIN);
        
        hw.hood.setPosition(Range.clip(hoodPos, HOOD_MIN, HOOD_MAX));

        // Calculate recommended RPM
        recommendedRPM = MIN_RPM + (normalized * (MAX_RPM - MIN_RPM));
        recommendedRPM = Range.clip(recommendedRPM, MIN_RPM, MAX_RPM);
    }

    public void setPosition(double position) {
        hw.hood.setPosition(Range.clip(position, HOOD_MIN, HOOD_MAX));
    }

    public double getDistance() {
        return currentDistance;
    }

    public double getRecommendedRPM() {
        return recommendedRPM;
    }
}
