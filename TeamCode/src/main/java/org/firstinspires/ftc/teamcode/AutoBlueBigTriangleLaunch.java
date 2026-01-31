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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@Autonomous(name = "AutoBlueBigTriangleLaunch", group = "StarterBot")
@Disabled
public class AutoBlueBigTriangleLaunch extends LinearOpMode {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 900;
    final double LAUNCHER_MIN_VELOCITY = 850;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcherR = null;
    private DcMotorEx launcherL = null;
    Servo liftServo;
    Lifter liftArm;
    DcMotorEx spinMotor;
    Spinner spinner;

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");
        launcherR = hardwareMap.get(DcMotorEx.class, "RightFlyWheel");
        launcherL = hardwareMap.get(DcMotorEx.class, "LeftFlyWheel");
        liftServo = hardwareMap.get(Servo.class,"LifterServo");
        liftArm = new Lifter(liftServo);
        spinMotor = hardwareMap.get(DcMotorEx.class, "PlateRotator");
        spinner = new Spinner(spinMotor);

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
//        launcherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);
//        launcherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
        waitForStart();
        driveForward(-0.7);
        sleep(1000);
        driveForward(0);

        //CODE FOR SHOOT
        for(int i=0; i<3; i++) {
            launcherR.setVelocity(LAUNCHER_TARGET_VELOCITY);
            launcherL.setVelocity(LAUNCHER_TARGET_VELOCITY);
            while (launcherL.getVelocity() < LAUNCHER_MIN_VELOCITY ||
                    launcherR.getVelocity() < LAUNCHER_MIN_VELOCITY) {
                telemetry.addLine("waiting for velocity");
            }
            liftArm.liftAndMoveBack();
            telemetry.addData("Current position B4: ", spinner.getPosition());
            spinner.rotate(120);
            telemetry.addData("Current position after: ", spinner.getPosition());
            sleep(2500);
        }
        driveForward(-0.67);
        sleep(400);
        driveForward(0);
        strafeLeft(0.5);
        sleep(500);
        strafeLeft(0);
//        rotateLeft(-0.5);
//        sleep(500);
//        rotateLeft(0);
//        driveForward(0.5);
//        sleep(400);
//        driveForward(0);
       /*
       * Moves our robot backwards and strafes to the inside of the triangle.
       * V1:FW -0.67 for 1000ms, Strafe Left -0.67 for 1000ms (default option)
       * V2:Fw -0.55 for 1000ms, Strafe Left -0.67 for 750ms(this moves the bot a shorter distance)
       * V3:FW -0.67 for 600ms, Strafe Left -0.55 for 800ms (this moves us a small ammount)
       * V4:FW -0.88 for 580ms, Strafe Left -0.88 for 480ms (Faster option)
        */
    }

    public void driveForward(double speed) {
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);
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
}

