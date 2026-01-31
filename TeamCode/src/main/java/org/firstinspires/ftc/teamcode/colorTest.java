package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "colorTest", group = "StarterBot")
@Disabled
public class colorTest extends OpMode {
    ColorDetector data;
    @Override
    public void init() {
        data = new ColorDetector(hardwareMap);
    }

    @Override
    public void loop() {
        data.getDetectedColor(telemetry);
    }
}
