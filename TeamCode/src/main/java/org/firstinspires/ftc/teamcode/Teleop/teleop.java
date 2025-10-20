package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp
public class teleop extends LinearOpMode {enum AutoState { IDLE, ROTATING, SETTLING, PUSH, RETRACT }
    AutoState autoState = AutoState.IDLE;
    boolean isAutoRunning = false;
    int[] positions= {0,1200,2400};
    ElapsedTime autoTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor magazine = hardwareMap.dcMotor.get("magazine");
        Servo servo = hardwareMap.servo.get("servo");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reverse the left side motors.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Constants

        boolean intakeOn = false;
        boolean intakeReverse=false;
        boolean PusherScore = false;
        boolean ShooterRunning = false;
        double shooterVelocity=-1500;
        int index=0;


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
            if (gamepad1.dpadUpWasPressed() && autoState == AutoState.IDLE) {
                index++;
                if (index >= positions.length) index = 0;

                magazine.setTargetPosition(positions[index]);
                magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                magazine.setPower(1);

                autoState = AutoState.ROTATING;
                isAutoRunning = true;
            }
            switch (autoState) {
                case ROTATING:
                    if (!magazine.isBusy()) {   // reached new slot
                        autoTimer.reset();
                        autoState = AutoState.SETTLING;
                    }
                    break;

                case SETTLING:
                    if (autoTimer.milliseconds() > 150) { // Pause
                        servo.setPosition(0.4);           // push to shoot
                        autoTimer.reset();
                        autoState = AutoState.PUSH;
                    }
                    break;

                case PUSH:
                    if (autoTimer.milliseconds() > 200) { // Pause
                        servo.setPosition(0.9);           // retract
                        autoTimer.reset();
                        autoState = AutoState.RETRACT;
                    }
                    break;

                case RETRACT:
                    if (autoTimer.milliseconds() > 150) { // allow retraction
                        autoState = AutoState.IDLE;
                        isAutoRunning = false;
                    }
                    break;

                case IDLE:
                default:
                    break;
            }

            if (gamepad1.squareWasPressed()) {
                ShooterRunning = !ShooterRunning;
                if (ShooterRunning) {
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooter.setVelocity(shooterVelocity); // ? Pid to control velocity
                    shooter.setVelocityPIDFCoefficients(1.0,1.0,1.0,1.0);
                } else {
                    shooter.setPower(0);
                }
            }
            // Just edit positions array once obtained
            if(gamepad1.triangleWasPressed())
            {
                index++;
                if(index>=positions.length)
                {
                    index=0;
                }
                magazine.setTargetPosition(positions[index]);
                magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                magazine.setPower(1);
            }

            if(gamepad1.dpadDownWasPressed())
            {
                index--;
                if(index<0)
                {
                    index=positions.length-1;
                }
                magazine.setTargetPosition(positions[index]);
                magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                magazine.setPower(1.0);
            }
            if (gamepad1.dpadUpWasPressed() && autoState == AutoState.IDLE) {
                // start automated cycle
                index++;
                if (index >= positions.length) index = 0;

                magazine.setTargetPosition(positions[index]);
                magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                magazine.setPower(1);

                autoState = AutoState.ROTATING;
                isAutoRunning = true;
            }
            telemetry.addData("Magazine Index", index);
            telemetry.addData("Auto State", autoState);
            telemetry.addData("Servo Pos", servo.getPosition());
            telemetry.update();
            if (gamepad1.circleWasPressed()) {
                PusherScore = !PusherScore;
                if (PusherScore) {
                    servo.setPosition(.4);
                } else {
                    servo.setPosition(.9);
                }
            }
            if(gamepad1.leftBumperWasPressed())
            {
                intakeReverse=!intakeReverse;
                intakeOn=false;
                if(intakeReverse)
                {
                    intake.setPower(0.6);
                }
                else
                {
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
