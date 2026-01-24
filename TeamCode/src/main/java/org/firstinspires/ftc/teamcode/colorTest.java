package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "colorTest", group = "StarterBot")
public class colorTest extends OpMode {
    colorTestData data = new colorTestData();
    @Override
    public void init() {
        data.init(hardwareMap);
    }

    @Override
    public void loop() {
        data.getDetectedColor(telemetry);
    }
}
