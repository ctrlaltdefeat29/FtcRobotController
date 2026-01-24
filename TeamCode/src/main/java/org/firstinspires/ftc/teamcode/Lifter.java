package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


//@TeleOp
public class Lifter {// extends OpMode {
    //lifter = the bootkicker that lifts the ball into the shooter
    private final Servo lifter;
    //START_POSITION = The position where the lifter starts
    final double START_POSITION = 0.01;
    //STOP_POSITION =  The position where the lifter stops
    final double STOP_POSITION = 0.4;

    public Lifter(Servo lifter){
        this.lifter = lifter;
        this.lifter.setPosition(START_POSITION);
    }


    public void liftAndMoveBack(){
        lifter.setPosition(STOP_POSITION);
        // The lifter waits for 2 seconds (2 * 1000 miliseconds = 2 seconds )
        // and then the lifter returns to the START_POSITION because it's a loop
        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
//        do {
//            try {
//                sleep(1);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//        } while (!(lifter.getPosition() >= STOP_POSITION));
        lifter.setPosition(START_POSITION);
        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}