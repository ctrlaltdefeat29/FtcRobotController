package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

//@Disabled

public class DistanceDetector{

    private AprilTagDetector vision;
    private Telemetry telemetry;

    public DistanceDetector(HardwareMap hardwareMap, Telemetry tel) {
       vision = new AprilTagDetector(hardwareMap);
       telemetry = tel;
    }
    final double TARGET_DISTANCE = 20.2;

    public void update()
    {
        vision.update();
    }
    public double distance_assess(AprilTagDetection detection) {
        AprilTagPoseFtc pos = detection.ftcPose;
        double distance = Math.sqrt(pos.x * pos.x + pos.y * pos.y);
//        telemetry.addData("Distance is", distance);
//        telemetry.addData("pos x", pos.x);
//        telemetry.addData("pos y", pos.y);
        return distance;
    }

    public double distanceAssessFromBlue() {
        if (vision.seesTag(20)) {
            AprilTagDetection detection = vision.getTag(20);
            double distance = distance_assess(detection);
            //correlate distance to the speed of the flywheel
            //if distance >= 35 then shoot normal (target velocity+ 0.3*distance)
            //else if distance < target distance then shoot lighter (target velocity - 0.3*distance)
            telemetry.addData("Blue Goal", distance );
            return distance;
        }
            return 0;
    }

    public double check_motif() {
        if (vision.seesTag(21)) {
            // Tag 21- GPP motif. Set motif to Green, Purple, Purple
            AprilTagDetection detection = vision.getTag(21);
//            int Motif1 = 0;
//            int Motif2 = 1;
//            int Motif3 = 1;
            double pattern = 0.11;
            return pattern;
        } else if (vision.seesTag(22)) {
            // Tag 22- PGP motif. Set motif to Purple, Green, Purple
            AprilTagDetection detection = vision.getTag(22);
//            int Motif1 = 1;
//            int Motif2 = 0;
//            int Motif3 = 1;
            double pattern = 1.01;
            return pattern;
        } else if (vision.seesTag(23)) {
            // Tag 23- PPG motif. Set motif to Purple, Purple, Green
            AprilTagDetection detection = vision.getTag(23);
//            int Motif1 = 1;
//            int Motif2 = 1;
//            int Motif3 = 0;
            double pattern = 1.10;
            return pattern;
        }
        return 0;
        //Tag 20- Blue Goal post
//        if (vision.seesTag(20)) {
//            AprilTagDetection detection = vision.getTag(20);
//            double distance = distance_assess(detection);
//            //correlate distance to the speed of the flywheel
//            //if distance > target distance then shoot stronger (target velocity+ 0.3*distance)
//            //else if distance < target distance then shoot lighter (target velocity - 0.3*distance)
//            telemetry.addData("Blue Goal", distance );
//        }
//        //Tag 24- Red Goal post
//        if (vision.seesTag(24)) {
//            AprilTagDetection detection = vision.getTag(20);
//            distance_assess(detection);
//            double distance = distance_assess(detection);
//            telemetry.addData("Red Goal", distance);
//        } else {
//            telemetry.addLine("No tags");
//            //if Mi<distance<Ma then set speed to _
//        }


    }
//    if (gamepad1.righttrigger){
//        while(colorsensor != motif1){
//            spinner.rotate(120);
//        }
//        while(colorsensor != motif2){
//            spinner.rotate(120);
//        }
//        while(colorsensor != motif3){
//            spinner.rotate(120);
//        }

    }