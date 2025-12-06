package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.concurrent.TimeUnit;

@TeleOp
public class teleop extends LinearOpMode {
    boolean isAutoRunning = false;
    int positions = 0;

    ElapsedTime autoTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor magazine = hardwareMap.dcMotor.get("magazine");
        Servo servo = hardwareMap.servo.get("servo");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        magazine.setTargetPosition(0);
        magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reverse the left side motors.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor.enableLed(true);  // Turn on sensor's LED

        // Constants
        boolean intakeOn = false;
        boolean intakeReverse = false;
        boolean PusherScore = false;
        boolean ShooterRunning = false;
        boolean autoIntake = false;

        boolean spun = false;
        boolean wasTan = false;
        double shooterVelocity = -960;
        double shooterVelocityFar = -1300;
        int rotationTimes = 0;

        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //telemetry.update();
            double y = -gamepad1.left_stick_y; //front-back;  remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; //left-right
            double rx = gamepad1.right_stick_x;//rotation
            y = y * Math.abs(y);
            x = x * Math.abs(x);
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            //double frontLeftPower1 = frontLeftPower * Math.abs(frontLeftPower);
            //double backLeftPower1 = backLeftPower * Math.abs(backLeftPower);
            //double frontRightPower1 = frontRightPower * Math.abs(frontRightPower);
            //double backRightPower1 = backRightPower * Math.abs(backRightPower);
            //slow mode
            if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            } else {
                frontLeftMotor.setPower(frontLeftPower / 3);
                backLeftMotor.setPower(backLeftPower / 3);
                frontRightMotor.setPower(frontRightPower / 3);
                backRightMotor.setPower(backRightPower / 3);
            }
            // Automated shooting
            if (gamepad1.dpad_up) {

                positions -= 250;                      // spin to next ball
                servo.setPosition(.05);
                sleep(300);
                servo.setPosition(0.6);
                sleep(400);


                magazine.setTargetPosition(positions);
                magazine.setPower(0.7);


            }
        if (gamepad1.crossWasPressed())
            {
                autoIntake = !autoIntake;
                spun=false;

            // Commented out for now test how it works    spun = false;
                if (autoIntake)
                {
                    int r = colorSensor.red();
                    int g = colorSensor.green();
                    int b = colorSensor.blue();

                    boolean isTan = g > 50 && g < 80 && r > 30 && r < 50 && b > 30 && b < 55;
                    telemetry.addData("Red", r);
                    telemetry.addData("Green", g);
                    telemetry.addData("Blue", b);
                    telemetry.addData("Is Tan?", isTan);
                    telemetry.update();
                    if (isTan) {
                        intake.setPower(0.8);
                        timer.reset();
                        spun = false;
                    }
                    else {
                        intake.setPower(0);
                        if (!spun && timer.milliseconds() > 150) {
                            if (rotationTimes%3 != 2) {
                                positions = positions - 250;
                                magazine.setTargetPosition(positions);
                                spun = true;
                                rotationTimes = rotationTimes + 1;
                            }
                        }
                    }
                }
     //           else
       //         {
         //           intake.setPower(0);
          //      }
            } //else {
//                    intake.setPower(0);
//                }

            if (gamepad1.squareWasPressed()) {
                    ShooterRunning = !ShooterRunning;
                    if (ShooterRunning) {
                        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        shooter.setVelocity(shooterVelocity); // ? Pid to control velocity

                    } else {
                        shooter.setPower(0);
                    }
                }
                // Just edit positions array once obtained
                if (gamepad1.triangleWasPressed()) {

                    positions = positions - 250;
                    magazine.setTargetPosition(positions);

                    magazine.setPower(0.3);
                }
                if (gamepad1.dpadRightWasPressed()) {
                    positions = positions + 30;
                    magazine.setTargetPosition(positions);
                    magazine.setPower(1.0);
                }
                if (gamepad1.dpadLeftWasPressed()) {

                    positions = positions - 30;

                    magazine.setTargetPosition(positions);
                    magazine.setPower(1.0);
                }

                //              telemetry.addData("Current Position", magazine.getCurrentPosition());
         /*

          */
            if (gamepad1.right_stick_button) {
                ShooterRunning = !ShooterRunning;
                if (ShooterRunning) {
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooter.setVelocity(shooterVelocityFar); // ? Pid to control velocity

                } else {
                    shooter.setPower(0);
                }
            }
                if (gamepad1.circleWasPressed()) {
                    servo.setPosition(0.6);
                    sleep(100);
                    servo.setPosition(.05);
                    sleep(400);
                    servo.setPosition(0.6);
                    sleep(100);
                    if (rotationTimes == 2%3){
                        rotationTimes = rotationTimes + 1;
                    }



                }


                if (gamepad1.leftBumperWasPressed()) {
                    intakeReverse = !intakeReverse;
                    intakeOn = false;
                    if (intakeReverse) {
                        intake.setPower(0.6);
                    } else {
                        intake.setPower(0);
                    }

                }
                if (gamepad1.rightBumperWasPressed()) {
                    intakeOn = !intakeOn;
                    if (intakeOn) {
                        intake.setPower(-0.6);
                    } else {
                        intake.setPower(0);
                    }

                }
            }
        }
    }


