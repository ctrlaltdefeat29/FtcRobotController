package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagDetector {


    //Used to get vision data from camera

    private AprilTagProcessor processor;


    //Used to control the "status" of the camera (on/off, streaming/paused)

    private VisionPortal visionPortal;


    //List of all of the current detections

    private List<AprilTagDetection> detections;
    private AprilTagDetection[] detectionMap;

    public AprilTagDetector(HardwareMap hardwareMap) {
        processor = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                .addProcessor(processor)
                .build();
    }
    public void update() {
        // getting all the apriltags seen by the camera
        detections = processor.getDetections();
        detectionMap = new AprilTagDetection[5];
        for (AprilTagDetection detection : detections) {
            // set the slot at id - 20 to this detection variable
            if(detection != null && detection.id >= 20 && detection.id < 25) {
                detectionMap[detection.id - 20] = detection;
            }
        }
    }
    //Check if the tag is visible
    public boolean seesTag(int id) {
        return detectionMap[id-20] != null;
    }

    public AprilTagDetection getTag(int id) {
        return detectionMap[id-20];

    }
}