package org.firstinspires.ftc.teamcode.Teleop;
import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp
public class teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        //DcMotor magazine = hardwareMap.dcMotor.get("magazine");
        Servo servo = hardwareMap.servo.get("servo");


        // Reverse the left side motors.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //clawRotate.setDirection(Servo.Direction.FORWARD);
        //grabber.setDirection(Servo.Direction.FORWARD);

//        slideExtend.setDirection(DcMotorSimple.Direction.REVERSE);
//        slideExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideRotate.setDirection(DcMotorSimple.Direction.FORWARD);

//        int RotateStartPos = 0;
//        int RotateMediumPos = 1230;
//        int RotateMaxPos = 5000;
//        int RotateHangPos = 6000;
//        int wall_pickup_extend = 100;
//        int elementRotateStart = 4500;
//        int elementRotateEnd = 2800;
//        int elementExtendStart = 600;
//        int elementExtendEnd = 0;
//        double grabber_open = .65;
//        double grabber_close = .1;
//        double grabber_up = .35;
//        double grabber_down = .6;
//        double grabber_hang = 0.5;
//        int maxSlideExtend = 1900;


//        slideRotate.setTargetPosition(RotateStartPos);
//        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        grabber.setPosition(grabber_open);
//        clawRotate.setPosition(.6); //FIXME

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Get the current position of the motor
//            double rotpos = slideRotate.getCurrentPosition();
//            double extpos = slideExtend.getCurrentPosition();

            // Show the position of the motor on telemetry
            //telemetry.addData("Rotational encoder position", rotpos);
            //telemetry.addData("Extension encoder position", extpos);

            //telemetry.update();
            double y = -gamepad1.left_stick_y; //front-back;  remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; //left-right
            double rx = gamepad1.right_stick_x;//rotation
            y = y*Math.abs(y);
            x = x*Math.abs(x);
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
                frontLeftMotor.setPower(frontLeftPower/3);
                backLeftMotor.setPower(backLeftPower/3);
                frontRightMotor.setPower(frontRightPower/3);
                backRightMotor.setPower(backRightPower/3);
            }
            if (gamepad1.squareWasPressed()){
                shooter.setPower(-.5);
                 // resetRuntime();
              //  while(time<2)
              //  {
               //     shooter.setPower(-0.5)
              //  }
            }
           if(gamepad1.squareWasReleased())
            {
                shooter.setPower(0);
            }


            if (gamepad1.circleWasPressed()){
                servo.setPosition(.4);
            }
            if (gamepad1.crossWasPressed()){
                servo.setPosition(.9);
            }
            /*

             */
    //   boolean intakeRunning=false;
            if (gamepad1.rightBumperWasPressed()) {

                intake.setPower(-.4);

            }
            if(gamepad1.rightBumperWasReleased())
            {
                intake.setPower(0);
            }
            // **********************************
            //special buttons
            //starting position: linear slide horizontal, fully retracted
//            if (gamepad2.circle) {
//                slideExtend.setTargetPosition(0);
//                slideRotate.setTargetPosition(RotateStartPos);
//                slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideExtend.setPower(1.0);
//                slideRotate.setPower(1.0);
//            }
            // for picking specimens from the wall
//            if (gamepad2.cross) {
//                slideRotate.setTargetPosition(RotateMediumPos);
//                slideExtend.setTargetPosition(wall_pickup_extend);
//                slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideExtend.setPower(1.0);
//                slideRotate.setPower(1.0);
//                clawRotate.setPosition(grabber_up);
//                grabber.setPosition(grabber_open);
            }
            // maximal slide rotation
//            if (gamepad2.triangle) {
//                slideRotate.setTargetPosition(RotateMaxPos);
//                slideRotate.setPower(1.0);
//            }
//            //one-button sequence for placing specimens on the bar
//            if (gamepad2.square) {
//                //setToPosition
//                clawRotate.setPosition(grabber_hang);
//                slideExtend.setTargetPosition(elementExtendStart);
//                slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideRotate.setTargetPosition(elementRotateStart);
//                slideExtend.setPower(1.0);  // Move towards target
//                slideRotate.setPower(1.0);
//                while (slideExtend.isBusy() || slideRotate.isBusy()) {
//                    //just wait
//                }
                //now, rotate slide  down to hang the specimen
//                slideRotate.setTargetPosition(elementRotateEnd);
//                clawRotate.setPosition(grabber_down);
//                slideExtend.setTargetPosition(elementExtendEnd);
            }
//            // Dpad: robot ascent/hanging
//            if (gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_right) {
//                slideExtend.setTargetPosition(1700);
//                slideRotate.setTargetPosition(RotateHangPos);
//                slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideExtend.setPower(1);
//                slideRotate.setPower(1);
//                while (slideRotate.isBusy() || slideExtend.isBusy()) {
//                    //just wait
//                }
//
//                slideExtend.setTargetPosition(1100);
//                while (slideExtend.isBusy()) {
//                    //just wait
//                }
//                slideRotate.setTargetPosition(0);
//            }
//
//            // **********************************
//            // Claw controls
//            if (gamepad2.right_bumper) {
//                grabber.setPosition(grabber_open);
//            }
//            if (gamepad2.left_bumper) {
//                grabber.setPosition(grabber_close);
//            }
//            if (gamepad1.left_bumper) {
//                clawRotate.setPosition(grabber_up);
//            }
//            if (gamepad1.right_bumper) {
//                clawRotate.setPosition(grabber_down);
//            }
//            // **********************************
//            //manual slide extension
//            double slideExtendPower =  -gamepad2.right_stick_y;
//            extpos = slideExtend.getCurrentPosition();
//            rotpos = slideRotate.getCurrentPosition();
//            //FIXME: defined constants
//            if (rotpos > 1200) {
//                maxSlideExtend = 2320;
//            } else {
//                maxSlideExtend = 1700;
//            }
//            //if slide extend motor was  in RUN_TO_POSITION mode but driver presses manual override
//            if (slideExtend.getMode() == DcMotor.RunMode.RUN_TO_POSITION && Math.abs(slideExtendPower)>0.3){
//                //override - switch to manual mode
//                slideExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//            if (slideExtend.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
//                //manual control
//                if (slideExtendPower > 0 && extpos < maxSlideExtend) {
//                    slideExtend.setPower(slideExtendPower);
//                } else if (slideExtendPower < 0 && extpos > 20){
//                    slideExtend.setPower(slideExtendPower);
//                }  else {
//                    slideExtend.setPower(0);
//                }
//            }




        //}
    //}

}