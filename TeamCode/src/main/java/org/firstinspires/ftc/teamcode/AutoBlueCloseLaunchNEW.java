/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoBlueCloseLaunchNEW", group = "StarterBot")
//@Disabled
public class AutoBlueCloseLaunchNEW extends LinearOpMode {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1000;
    final double LAUNCHER_MIN_VELOCITY = 990;

    // Declare OpMode members.
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx launcherR = null;
    private DcMotorEx launcherL = null;
    private DcMotorEx intakeMotor = null;
    Servo liftServo;
    Lifter liftArm;
    DcMotorEx spinMotor;
    Spinner spinner;

    Intaker intaker;
    private DistanceDetector detector;
    private ColorDetector colorDetector;

    private OdometryMovement odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "BackRightDrive");
        launcherR = hardwareMap.get(DcMotorEx.class, "RightFlyWheel");
        launcherL = hardwareMap.get(DcMotorEx.class, "LeftFlyWheel");
        liftServo = hardwareMap.get(Servo.class,"LifterServo");
        liftArm = new Lifter(liftServo);
        spinMotor = hardwareMap.get(DcMotorEx.class, "PlateRotator");
        spinner = new Spinner(spinMotor);
        detector = new DistanceDetector(hardwareMap, telemetry);
        colorDetector = new ColorDetector(hardwareMap);
        odometry = new OdometryMovement(leftBackDrive, rightBackDrive, leftFrontDrive, rightFrontDrive, this);
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        intaker = new Intaker(intakeMotor);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);
//        launcherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        //     launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
        waitForStart();

        launcherR.setVelocity(LAUNCHER_TARGET_VELOCITY +20);
        launcherL.setVelocity(LAUNCHER_TARGET_VELOCITY +20);

        odometry.moveBackward(48);

        //SHOOT
        Shoot();

        long turnTimeMs = 1530;  // tune this for YOUR robot

        odometry.moveBackward(9);

        // right turn
        rotateLeft(-0.4);

        sleep(turnTimeMs);

        // Stop
        rotateLeft(0);

//        odometry.moveBackward(6);

        intaker.runAutoIntake();
        odometry.moveBackward(50);

        //intaker.backoutBall();

        //sleep(300);

//        sleep(200);

        odometry.moveForward(12);
        intaker.backoutBall();
        sleep(200);
        intaker.stopIntake();


        spinner.rotate(360);
//        spinner.rotate(240);

        odometry.moveForward(38);

        turnTimeMs = 1540;  // tune this for YOUR robot

        // Left turn
        rotateLeft(0.4);
        sleep(turnTimeMs);

        rotateLeft(0);

        Shoot();

        strafeLeft(0.4);
        sleep(1000);

        launcherR.setVelocity(0);
        launcherL.setVelocity(0);
    }


    public void strafeLeft(double speed) {
        leftFrontDrive.setPower(-speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(-speed);
    }
    public void rotateLeft(double speed){
        leftFrontDrive.setPower(-speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(-speed);
        rightBackDrive.setPower(speed);
    }

    private void Shoot() {
        for(int index=0; index<3; ++index) {
            while (launcherL.getVelocity() < LAUNCHER_MIN_VELOCITY ||
                    launcherR.getVelocity() < LAUNCHER_MIN_VELOCITY) {
                telemetry.addLine("waiting for velocity");
            }
            liftArm.liftAndMoveBack();
            spinner.rotate(120);
        }
    }
}

