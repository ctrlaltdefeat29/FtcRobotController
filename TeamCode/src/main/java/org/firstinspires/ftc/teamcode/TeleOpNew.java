package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpNew", group = "StarterBot")
//@Disabled
public class TeleOpNew extends OpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    DcMotorEx launcherR;
    DcMotorEx launcherL;
    Servo liftServo;
    Lifter liftArm;
    DcMotorEx spinMotor;
    DcMotorEx intakeMotor;
    Intaker intaker;
    Spinner spinner;

    DistanceDetector detector;

    ColorDetector colorDetector;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    double velocity;
    double minVelocity;
    // must be more than 800, 700
    final double LAUNCHER_CLOSE_VELOCITY = 1000;
    final double LAUNCHER_MIN_CLOSE_VELOCITY = 990;

    final double LAUNCHER_FAR_VELOCITY = 1150;
    final double LAUNCHER_MIN_FAR_VELOCITY = 1140;
    final double STOP_SPEED = 0;

    private boolean intaking;

    //    private boolean intake = false;
    @Override
    public void init() {
        liftServo = hardwareMap.get(Servo.class,"LifterServo");
        liftArm = new Lifter(liftServo);
        spinMotor = hardwareMap.get(DcMotorEx.class, "PlateRotator");
        spinner = new Spinner(spinMotor);
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        intaker = new Intaker(intakeMotor);
        launcherR = hardwareMap.get(DcMotorEx.class, "RightFlyWheel");
        launcherL = hardwareMap.get(DcMotorEx.class, "LeftFlyWheel");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");
        detector = new DistanceDetector(hardwareMap, telemetry);
        colorDetector = new ColorDetector(hardwareMap);
        intaking = false;

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

//        launcherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        //      launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);
//        launcherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        //     launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);

        velocity = LAUNCHER_CLOSE_VELOCITY;
        minVelocity = LAUNCHER_MIN_CLOSE_VELOCITY;
    }

    @Override
    public void loop() {
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        runLauncher();
        detector.update();

        double dist = detector.distanceAssessFromBlue();
        if (dist > 75.0) {
            velocity = LAUNCHER_FAR_VELOCITY;
            minVelocity = LAUNCHER_MIN_FAR_VELOCITY;
        } else {
            velocity = LAUNCHER_CLOSE_VELOCITY;
            minVelocity = LAUNCHER_MIN_CLOSE_VELOCITY;
        }
        telemetry.addData("Distance: ", dist);
        telemetry.addData("R velocity", launcherR.getVelocity());
        telemetry.addData("L velocity", launcherL.getVelocity());

        if (gamepad1.y && !intaking) {
            telemetry.addLine("starting intake");
            intaker.runIntake();
            intaking = true;
        }

        if (gamepad1.x && intaking) {
            telemetry.addLine("stopping intake");
            intaker.stopIntake();
            spinner.rotate(120);
            intaking = false;
        }

        if(gamepad1.b && intaking) {
            telemetry.addLine("reversing intake");
            intaker.backoutBall();
        }

        if(!intaking) {
            if (gamepad1.left_bumper) {
                for (int i = 0; i < 3; i++) {
                    if (colorDetector.getDetectedColor(telemetry) == ColorDetector.detectedColor.UNKNOWN) {
                        if(i != 2) {
                            spinner.rotate(120);
                            try {
                                sleep(200);
                            } catch (InterruptedException e) {
                                throw new RuntimeException(e);
                            }
                        }
                        continue;
                    }
                    while (launcherL.getVelocity() < minVelocity ||
                            launcherR.getVelocity() < minVelocity) {
                        telemetry.addLine("waiting for velocity");
                    }
                    liftArm.liftAndMoveBack();
                    telemetry.addData("Current position B4: ", spinner.getPosition());
                    if(i!=2) {
                        spinner.rotate(120);
                        telemetry.addData("Current position after: ", spinner.getPosition());
                    }
                }
            }
            if (gamepad1.dpad_right) {
                telemetry.addData("Current position B4: ", spinner.getPosition());
                spinner.rotate(230);
                telemetry.addData("Current position after: ", spinner.getPosition());
            }

            if (gamepad1.dpad_left) {
                telemetry.addData("Current position B4: ", spinner.getPosition());
                spinner.rotate(250);
                telemetry.addData("Current position after: ", spinner.getPosition());
            }
        }

    }

    public void runLauncher(){
        launcherR.setVelocity(velocity);
        launcherL.setVelocity(velocity);
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
