package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


//the bootkicker that lifts the ball into the shooter
public class Lifter {
    private final Servo lifter;
    //START_POSITION = The position where the lifter starts
    final double START_POSITION = 0.01;
    //STOP_POSITION =  The position where the lifter stops
    final double STOP_POSITION = 0.28;
    //MID_POSITION =  The position where the lifter waits to align ball
    final double MID_POSITION = 0.15;

    public Lifter(Servo lifter){
        this.lifter = lifter;
        this.lifter.setPosition(START_POSITION);
    }
    public void liftAndMoveBack(){
        lifter.setPosition(MID_POSITION);
        try {
            sleep(350);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);}

        lifter.setPosition(STOP_POSITION);
        try {
            sleep(150);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        lifter.setPosition(START_POSITION);
        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}