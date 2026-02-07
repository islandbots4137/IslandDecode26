package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import static org.firstinspires.ftc.teamcode.Constants.TeleOpConstants.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class teleop extends LinearOpMode {
    private Limelight3A limelight3A;
    boolean lastPressed = false;
    int sequenceState = 0;
    boolean lastDpadUp = false;
    boolean indexing = false;
    boolean servoMoving = false;
    boolean requireRelease = false;
    private ElapsedTime sequenceTimer = new ElapsedTime();

    boolean autoTurnEnabled = false;

    DigitalChannel magLimitSwitch;
    ElapsedTime servoTimer = new ElapsedTime();
    boolean lastTriangleState = false;  // For edge detection
    boolean magazineRotating = false;
    boolean lastSquare = false;   // for edge detection


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        Servo aimLight = hardwareMap.get(Servo.class, "aimLight");
        magLimitSwitch = hardwareMap.get(DigitalChannel.class, "magSwitch"); // your config name
        magLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor magazine = hardwareMap.dcMotor.get("magazine");
        magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Servo servo = hardwareMap.servo.get("servo");
        magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime indexTimer = new ElapsedTime();
        magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         magazine.setTargetPosition(0);
        // Reverse the left side motors.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ElapsedTime timer = new ElapsedTime();
        GoBildaPinpointDriver odo;
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(0.2,-6.5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        odo.resetPosAndIMU();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        limelight3A.start();
        limelight3A.pipelineSwitch(8);
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; //front-back;  remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; //left-right
            double rx = gamepad1.right_stick_x;//rotation
            boolean runAutoTurn;
            boolean dpadUpPressed = gamepad1.dpad_up;
/*
            boolean square = gamepad2.squareWasPressed();
            if (square && !lastSquare) {
                autoTurnEnabled = !autoTurnEnabled;
            }
            lastSquare = square;

 */
            /*
            autoTurnEnabled = gamepad2.square;
            if (autoTurnEnabled) {
                LLResult llResult = limelight3A.getLatestResult();

                if (llResult != null && llResult.isValid()) {

                    List<LLResultTypes.FiducialResult> fiducials =
                            llResult.getFiducialResults();

                    if (!fiducials.isEmpty()) {

                        // pick first tag (or filter by ID)
                        LLResultTypes.FiducialResult tag = fiducials.get(0);

                        double tx = tag.getTargetXDegrees(); // left/right angle to tag
                        if (Math.abs(tx) > TX_DEADBAND) {
                            rx = tx * TAG_TURN_kP;
                            rx = Math.max(-MAX_RX, Math.min(MAX_RX, rx));
                        } else {
                            rx = 0.0; // already aligned
                        }
                    } else {
                        rx = 0.0; // no tag → don’t spin
                    }
                } else {
                    rx = 0.0;
                }
            }
            ;
*/
            telemetry.addData("rx", rx);

            odo.update();

            double headingDeg = odo.getHeading(AngleUnit.DEGREES);
            double headingRad = Math.toRadians(headingDeg);
            if (gamepad2.dpad_left) {
                headingSetpoint = 225;
                runAutoTurn = true;
            } else if (gamepad2.dpad_right) {
                headingSetpoint = 135;
                runAutoTurn = true;
            } else {
                runAutoTurn = false;
            }
            if (runAutoTurn) {

                double error = headingSetpoint - Math.toDegrees(headingRad);

                if (error > 180) {
                    error -= 360;
                } else if (error < -180) {
                    error += 360;
                }

                rx = 0.020 * error;
                rx = Math.min(Math.max(rx, -0.4), 0.4);
            }

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


           if (dpadUpPressed && !lastDpadUp && sequenceState == 0) {
            // Start the sequence
            magazine.setPower(magazinePower);
            positions -= 250;
            servo.setPosition(servoForward);
            sequenceTimer.reset();
            sequenceState = 1;
        }
        lastDpadUp = dpadUpPressed;

        // State 1: Wait 300ms, then move servo back
        if (sequenceState == 1 && sequenceTimer.milliseconds() >= 300) {
            servo.setPosition(servoBack);
            sequenceTimer.reset();
            sequenceState = 2;
        }

        // State 2: Wait 400ms, then set magazine target
        if (sequenceState == 2 && sequenceTimer.milliseconds() >= 400) {
            magazine.setTargetPosition(positions);
            sequenceState = 0;  // Back to idle
        }

            double lightPos = NO_TAG_POS;

            if(gamepad2.right_bumper) {
                LLResult llResult = limelight3A.getLatestResult();
                if (llResult != null && llResult.isValid()) {

                    List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

                    if (fiducials != null && !fiducials.isEmpty()) {

                        // pick tag closest to center
                        LLResultTypes.FiducialResult best = fiducials.get(0);
                        double bestAbsTx = Math.abs(best.getTargetXDegrees());

                        for (LLResultTypes.FiducialResult f : fiducials) {
                            double absTx = Math.abs(f.getTargetXDegrees());
                            if (absTx < bestAbsTx) {
                                bestAbsTx = absTx;
                                best = f;
                            }
                        }

                        double tx = best.getTargetXDegrees();

                        if (Math.abs(tx) <= ALIGN_THRESH_DEG) {
                            lightPos = GREEN_POS;       // aligned
                        } else {
                            lightPos = NOT_READY_POS;   // not aligned
                        }
                    }
                }
            }
            aimLight.setPosition(lightPos);

            if (gamepad1.dpadLeftWasPressed()) {
                magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                magazine.setPower(magazinePower);
                positions+=250;
                magazine.setTargetPosition(positions);
            }
            if (gamepad1.dpadRightWasPressed()) {
                magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                magazine.setPower(magazinePower);
                positions-=250;
                magazine.setTargetPosition(positions);

            }
            if (gamepad2.circleWasPressed()) {
                magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                magazine.setPower(magazinePower);
                positions+=10;
                magazine.setTargetPosition(positions);
            }
            if (gamepad2.triangleWasPressed()) {
                magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                magazine.setPower(magazinePower);
                positions -= 10;
                magazine.setTargetPosition(positions);
            }
            if (gamepad1.squareWasPressed()) {
                ShooterRunning = !ShooterRunning;
                if (ShooterRunning) {
                    shooter.setVelocity(shooterVelocity); // ? Pid to control velocity
                    shooter2.setVelocity(shooterVelocity);
                } else {
                    shooter.setVelocity(0);
                    shooter2.setVelocity(0);
                }
            }
            // Just edit positions array once obtained
            if (gamepad1.triangleWasPressed()) {
                if (!magazineRotating) {
                    magazineRotating = true;
                    magazine.setPower(magazinePower);
                }
            }
            if (magazineRotating && !magLimitSwitch.getState()) {
                magazine.setPower(0);
                magazineRotating = false;
            }
         /*   if (gamepad1.triangleWasPressed()) {

                magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                positions = positions-250;
                magazine.setTargetPosition(positions);
                magazine.setPower(magazinePower);
            }

          */
            if (gamepad1.right_stick_button) {
                ShooterRunning = !ShooterRunning;
                if (ShooterRunning) {
                    shooter.setVelocity(shooterVelocityFar); // ? Pid to control velocity
                    shooter2.setVelocity(shooterVelocityFar);
                } else {
                    shooter.setPower(0);
                }
            }
            if (gamepad1.circle && !servoMoving) {
                servo.setPosition(servoForward);
                servoTimer.reset();
                servoMoving = true;
            }

            // After 200ms, move servo back
            if (servoMoving && servoTimer.milliseconds() >= 200) {
                servo.setPosition(servoBack);
                servoMoving = false;
            }


            if (gamepad1.leftBumperWasPressed()) {
                intakeReverse = !intakeReverse;
                if (intakeReverse) {
                    intake.setPower(intakeOut);
                } else {
                    intake.setPower(0);
                }

            }
            if (gamepad1.rightBumperWasPressed()) {
                intakeOn = !intakeOn;
                if (intakeOn) {
                    intake.setPower(intakeIn);
                } else {
                    intake.setPower(0);
                }
            }
//            telemetry.addData("y x rx", "%.3f %.3f %.3f", y, x, rx);
//            telemetry.addData("FL FR", "%.3f %.3f", frontLeftPower, frontRightPower);
//            telemetry.addData("BL BR", "%.3f %.3f", backLeftPower, backRightPower);
//            telemetry.addData("light", lightPos );
//            telemetry.addData("Limit Switch isPressed", magazineLimitSwitch.isPressed());
//            telemetry.update();

        }
    }
}

