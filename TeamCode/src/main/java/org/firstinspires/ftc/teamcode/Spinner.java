package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spinner {
    private final DcMotorEx spinMotor;

    // PID constants (we may need to tune these)
    final double kP = 0.006;
    final double kD = 0.0005;
    final double maxPower = 0.2; // slightly lower for smoothness

    // Encoder ticks per full output shaft revolution
//    int TICKS_PER_REV = 384;
    int TICKS_PER_REV = 1425;
    public Spinner(DcMotorEx spinMotorVar) {
        spinMotor = spinMotorVar;
        spinMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spinMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        spinMotor.setPositionPIDFCoefficients(kP);
    }

    public double getPosition() {
        return spinMotor.getCurrentPosition();
    }

    public void rotate(double angleDegrees) {
        int ticksToMove = (int)((angleDegrees / 360.0) * TICKS_PER_REV);
        int targetPos = spinMotor.getCurrentPosition()+ticksToMove;
        spinMotor.setTargetPosition(targetPos);
        spinMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(0.5);
        while(spinMotor.isBusy())
        {
            if(targetPos - spinMotor.getCurrentPosition() < 50)
            {
                spinMotor.setPower(0.2);
            }
        }
//        spinMotor.setPower(0.1);
    }
}
