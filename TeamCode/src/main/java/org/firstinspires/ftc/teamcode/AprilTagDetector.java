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


    //Constructor - has to run before using anything in the file, sort of an "init" for our file

    public AprilTagDetector(HardwareMap hardwareMap) {
        // build our processor, you can configure it by adding .doX() inbetween the Builder() and .build()
        processor = new AprilTagProcessor.Builder().build();

        // build our processor
        visionPortal = new VisionPortal.Builder()
                // set the camera this portal uses, "Webcam 1" is the default camera name
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                // add our previously made processor to this camera portal, this will cause it to use this camera
                .addProcessor(processor)
                .build();
    }

    public void update() {
        // get all the apriltags seen by the camera, and put them in our list
        detections = processor.getDetections();
        // reset all the slots on our detectionMap
        detectionMap = new AprilTagDetection[5];
        // go through every item in the list and perform an "action" on it
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

    public List<AprilTagDetection> getAllDetections() {
        return detections;
    }


}