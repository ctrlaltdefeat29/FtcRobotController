package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp(name = "Cams2", group = "StarterBot")
@Disabled

public class Cams2 extends OpMode {

    private Cams1 vision;

    @Override
    public void init() {
        vision = new Cams1(hardwareMap);
    }
    final double TARGET_DISTANCE = 20.2;
    @Override
    public void loop() {
        vision.update();


        check_motif();


    }

    public double distance_assess(AprilTagDetection detection) {
        AprilTagPoseFtc pos = detection.ftcPose;
        double distance = Math.sqrt(pos.x * pos.x + pos.y * pos.y);
        telemetry.addData("Distance is", distance);
        telemetry.addData("pos x", pos.x);
        telemetry.addData("pos y", pos.y);
        return distance;
    }

    public void check_motif() {
        if (vision.seesTag(21)) {
            // Tag 21- GPP motif. Set motif to Green, Purple, Purple
            AprilTagDetection detection = vision.getTag(21);
            //distance_assess(detection);
            int Motif1 = 0;
            int Motif2 = 1;
            int Motif3 = 1;
            telemetry.addData("motif 1", Motif1);
            telemetry.addData("motif 2", Motif2);
            telemetry.addData("motif 3", Motif3);
        } else if (vision.seesTag(22)) {
            // Tag 22- PGP motif. Set motif to Purple, Green, Purple
            AprilTagDetection detection = vision.getTag(22);
            //distance_assess(detection);
            int Motif1 = 1;
            int Motif2 = 0;
            int Motif3 = 1;
            telemetry.addData("motif 1", Motif1);
            telemetry.addData("motif 2", Motif2);
            telemetry.addData("motif 3", Motif3);
        } else if (vision.seesTag(23)) {
            // Tag 23- PPG motif. Set motif to Purple, Purple, Green
            AprilTagDetection detection = vision.getTag(23);
            //distance_assess(detection);
            int Motif1 = 1;
            int Motif2 = 1;
            int Motif3 = 0;
            telemetry.addData("motif 1", Motif1);
            telemetry.addData("motif 2", Motif2);
            telemetry.addData("motif 3", Motif3);
        }
        //Tag 20- Blue Goal post
        if (vision.seesTag(20)) {
            AprilTagDetection detection = vision.getTag(20);
            double distance = distance_assess(detection);
            //correlate distance to the speed of the flywheel
            //if distance > target distance then shoot stronger (target velocity+ 0.3*distance)
            //else if distance < target distance then shoot lighter (target velocity - 0.3*distance)
            telemetry.addData("Blue Goal", distance );
        }
        //Tag 24- Red Goal post
        if (vision.seesTag(24)) {
            AprilTagDetection detection = vision.getTag(20);
            distance_assess(detection);
            double distance = distance_assess(detection);
            telemetry.addData("Red Goal", distance);
        } else {
            telemetry.addLine("No tags");
            //if Mi<distance<Ma then set speed to _
        }


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