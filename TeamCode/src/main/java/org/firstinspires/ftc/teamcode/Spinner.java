package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spinner {
    private final DcMotorEx spinMotor;

    // PID constants (Need to tune these)
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
//        spinMotor.setPositionPIDFCoefficients(kP); //NOTE: this doesnt use kD
        //+now try and use setTargetPosition, instead of the logic in rotate
    }

    public void setMotorSpeed (double speed) {
        //Values between -1.0 and 1.0
        spinMotor.setPower(speed);
    }
//    public double getMotorRevs() {
//        return spinMotor.getCurrentPosition() / TicksPerRev;// / ticksPerRev ; //13.7:1 gear reduction - 383.6 ticks per revolution
//    }

    public double getPosition() {
        return spinMotor.getCurrentPosition();
    }

    public void rotate(double angleDegrees) {
//        int startPos = spinMotor.getCurrentPosition();
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
        //        int targetPos = startPos + ticksToMove;
//
//        int lastError = 0;
//        int tolerance = 4; // ±2 ticks = ~1–2°
//
//        // soft acceleration parameters
//        double minPower = 0.05; // ensures motor overcomes deadband
//        int maxTimeMs = 2000; // safety timeout
//        long startTime = System.currentTimeMillis();
//
////        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        int deltaError = (startPos - prevPosition);
//        while (Math.abs(targetPos - spinMotor.getCurrentPosition()) > tolerance &&
//                System.currentTimeMillis() - startTime < maxTimeMs) {
//
//            int error = (targetPos - spinMotor.getCurrentPosition()) - deltaError;
//            deltaError = 0;
//            int derivative = error - lastError;
//
//            double power = kP * error + kD * derivative;
//
//            // ensure minimum power to overcome deadband
//            if (power > 0 && power < minPower)
//            {
//                power = minPower;
//            }
//            if (power < 0 && power > -minPower)
//            {
//                power = -minPower;
//            }
//
//            // cap power to max
//            power = Math.max(-maxPower, Math.min(maxPower, power));
//
//            spinMotor.setPower(power);
//            lastError = error;
//            prevPosition = spinMotor.getCurrentPosition();
//        }
//
//        spinMotor.setPower(0);
    }


//    public int rotate(int divFac, int breakPos, Telemetry telemetry)
//    {
//        telemetry.addData("Target position: ", spinMotor.getTargetPosition());
//        int delta = spinMotor.getTargetPosition() - spinMotor.getCurrentPosition();
////        spinMotor.setTargetPosition(breakPos+(int)TicksPerRev/divFac+delta); // rotates roughly 120 degrees
//        spinMotor.setTargetPosition(spinMotor.getCurrentPosition()+(int)TicksPerRev/divFac+delta); // rotates roughly 120 degrees
//        setMotorSpeed(0.25);
//        while(isBusy()){
////            try {
////                sleep(100);
////            } catch (InterruptedException e) {
////                throw new RuntimeException(e);
////            }
//            telemetry.addData("Current position Busy: ", getPosition());
//        }
//        telemetry.addData("Current position after Busy: ", spinMotor.getCurrentPosition());
//        setMotorSpeed(0.0);
//        telemetry.addData("Current position after Break: ", spinMotor.getCurrentPosition());
//        return  spinMotor.getCurrentPosition();
//    }
    public boolean isBusy()
    {
        return spinMotor.isBusy();
    }
}
