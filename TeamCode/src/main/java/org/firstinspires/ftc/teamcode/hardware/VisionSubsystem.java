package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class VisionSubsystem {
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    public VisionSubsystem(HardwareMap hardwareMap) {
        // Create the AprilTag processor with high-performance settings
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Create the vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .enableLiveView(true) 
                .setAutoStopLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    /**
     * Toggles the AprilTag processor (which controls annotations in the preview).
     * @param enabled True to show annotations and process tags, False to hide them.
     */
    public void setPreviewEnabled(boolean enabled) {
        visionPortal.setProcessorEnabled(aprilTag, enabled);
    }

    /**
     * Optimizes camera for the current lighting. 
     * Low exposure reduces motion blur.
     */
    public void setManualExposure(long exposureMs, int gain) {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    public int getGain() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return 0;
        return visionPortal.getCameraControl(GainControl.class).getGain();
    }

    /**
     * Finds the first visible AprilTag detection.
     * Returns null if no tags are seen.
     */
    public AprilTagDetection getFirstDetection() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections != null && !currentDetections.isEmpty()) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    return detection;
                }
            }
        }
        return null;
    }

    public void close() {
        visionPortal.close();
    }

    public List<AprilTagDetection> getAllDetections() {
        return aprilTag.getDetections();
    }
    
    public VisionPortal.CameraState getCameraState() {
        return visionPortal.getCameraState();
    }
}
