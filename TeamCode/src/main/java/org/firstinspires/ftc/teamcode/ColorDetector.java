package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorDetector {
NormalizedColorSensor colorSensor;
    public enum detectedColor{
    PURPLE,
    GREEN,
    UNKNOWN
    }
    public ColorDetector(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class, "ColorSensor");
    }
    public detectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double sum = colors.red + colors.green + colors.blue;
        double red = colors.red/sum;
        double green = colors.green/sum;
        double blue = colors.blue/sum;

        if(green>red && green>blue && green>=0.42){
            telemetry.addLine("GREEN");
            return detectedColor.GREEN;
        }
        if(blue>red && blue>green){
           telemetry.addLine("PURPLE");
            return detectedColor.PURPLE;
        }
        telemetry.addLine("UNKNOWN");
        return detectedColor.UNKNOWN;
    }
}
