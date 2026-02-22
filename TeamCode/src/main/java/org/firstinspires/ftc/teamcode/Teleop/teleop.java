package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    private static final int SEQ_IDLE = 0;
    private static final int SEQ_FIRING = 1;
    private static final int SEQ_SERVO_RETRACT = 2;
    private static final int SEQ_MAG_ENCODER = 3;
    private static final int SEQ_MAG_CREEP = 4;
    int seqState = SEQ_IDLE;
    ElapsedTime seqTimer = new ElapsedTime();
    ElapsedTime limelightTimer = new ElapsedTime();
    boolean limelightActive = false;
    boolean lastDpadUp = false;
    boolean indexing = false;
    boolean servoMoving = false;
    boolean requireRelease = false;
    boolean magazineBraking = false;
    boolean magazineRotating = false;
    boolean magazineCreeping = false;

    boolean clearedMagnet = false;
    boolean magazineAligning = false;

    ElapsedTime brakeTimer = new ElapsedTime();
    private ElapsedTime sequenceTimer = new ElapsedTime();

    ElapsedTime magazineTimer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();

    boolean autoTurnEnabled = false;

    DigitalChannel magLimitSwitch;
    ElapsedTime servoTimer = new ElapsedTime();
    boolean lastTriangleState = false;  // For edge detection
    boolean lastSquare = false;   // for edge detection


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        Servo aimLight = hardwareMap.get(Servo.class, "aimLight");
        magLimitSwitch = hardwareMap.get(DigitalChannel.class, "magSwitch"); // your config name
        magLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(10, 3, 0, 12);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
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
        waitForStart();
        limelight3A.start();
        limelight3A.pipelineSwitch(8);
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Add this at the top of your loop to monitor
            telemetry.addData("Loop time", timer.milliseconds());
            timer.reset();
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
            boolean shooterReady = ShooterRunning
                    && Math.abs(shooterVelocity - shooter.getVelocity()) < 50
                    && Math.abs(shooterVelocity - shooter2.getVelocity()) < 50;

// ---- Start sequence on dpad_up ----
            if (gamepad1.dpad_up && !lastDpadUp && seqState == SEQ_IDLE && shooterReady) {
                // Fire the servo
                servo.setPosition(servoForward);
                seqTimer.reset();
                seqState = SEQ_FIRING;
            }
            lastDpadUp = gamepad1.dpad_up;

// ---- State 1: Wait for ball to leave, then retract servo ----
            if (seqState == SEQ_FIRING && seqTimer.milliseconds() >= 200) {
                servo.setPosition(servoBack);
                seqTimer.reset();
                seqState = SEQ_SERVO_RETRACT;
            }

// ---- State 2: Servo retracted, start magazine encoder move ----
            if (seqState == SEQ_SERVO_RETRACT && seqTimer.milliseconds() >= 300) {
                magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                magazine.setTargetPosition(-230); // most of the way to next slot — tune this
                magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                magazine.setPower(0.35);
                seqState = SEQ_MAG_ENCODER;
            }

// ---- State 3: Encoder move done, switch to slow creep ----
            if (seqState == SEQ_MAG_ENCODER && !magazine.isBusy()) {
                magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                magazine.setPower(-0.12); // slow creep toward magnet — tune direction & speed
                seqState = SEQ_MAG_CREEP;
            }

// ---- State 4: Magnet found, stop and go idle ----
            if (seqState == SEQ_MAG_CREEP && !magLimitSwitch.getState()) {
                magazine.setPower(0);
                magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                seqState = SEQ_IDLE;
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
                positions+=20;
                magazine.setTargetPosition(positions);
            }
            if (gamepad2.triangleWasPressed()) {
                magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                magazine.setPower(magazinePower);
                positions -= 20;
                magazine.setTargetPosition(positions);
            }
            if (gamepad1.squareWasPressed()) {
                ShooterRunning = !ShooterRunning;
            }

// Continuously command velocity every loop
            if (ShooterRunning) {
                shooter.setVelocity(shooterVelocity);
                shooter2.setVelocity(shooterVelocity);
            } else {
                shooter.setVelocity(0);
                shooter2.setVelocity(0);
            }

       /*
            if (gamepad1.triangleWasPressed()) {
                if (!magazineRotating) {
                    magazineRotating = true;
                    magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    magazine.setPower(-0.4);

                    // Only need to clear if we're currently ON a magnet
                    if (!magLimitSwitch.getState()) {
                        clearedMagnet = false; // on a magnet, need to leave it first
                    } else {
                        clearedMagnet = true;  // not on a magnet, already cleared
                        magazine.setPower(-0.08); // go slow since next magnet could be close
                    }
                }
            }

            // Stage 1: Leave current magnet, then slow down
            if (magazineRotating && !clearedMagnet && magLimitSwitch.getState()) {
                clearedMagnet = true;
                magazine.setPower(-0.12);
            }
            // Stage 2: We passed the magnet — stop and start reversing
            if (magazineRotating && clearedMagnet && !magLimitSwitch.getState()) {
                magazine.setPower(0);
                magazineRotating = false;
                clearedMagnet = false;
                magazineAligning = true;
            }

// Stage 3: Creep backward until sensor detects magnet
            if (magazineAligning && magLimitSwitch.getState()) {
                magazine.setPower(0.06); // slow reverse, tune this value
            }

            if (magazineAligning && !magLimitSwitch.getState()) {
                magazine.setPower(0);
                magazineAligning = false;
            }
            telemetry.addData("Switch", magLimitSwitch.getState());
            telemetry.addData("Rotating", magazineRotating);
            telemetry.addData("Cleared", clearedMagnet);
            telemetry.update();

        */
            // Stage 1: Start encoder move (most of the distance)
            if (gamepad1.triangleWasPressed() && !magazineRotating && !magazineCreeping) {
                magazineRotating = true;
                magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                magazine.setTargetPosition(-230); // tune this — slightly SHORT of the magnet
                magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                magazine.setPower(-0.35);
            }

// Stage 2: Encoder move done — switch to slow creep forward
            if (magazineRotating && !magazine.isBusy()) {
                magazineRotating = false;
                magazineCreeping = true;
                magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                magazine.setPower(-0.12); // slow forward (negative = forward), tune this
            }

// Stage 3: Sensor found — stop
            if (magazineCreeping && !magLimitSwitch.getState()) {
                magazine.setPower(0);
                magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                magazineCreeping = false;
            }
            /*   if (gamepad1.triangleWasPressed()) {

                magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                positions = positions-250;
                magazine.setTargetPosition(positions);
                magazine.setPower(magazinePower);
            }

          */
            if (gamepad1.right_stick_button) {
                ShooterRunningFast = !ShooterRunningFast;
            }
            if (ShooterRunningFast) {
                shooter.setVelocity(shooterVelocityFar);
                shooter2.setVelocity(shooterVelocityFar);
            } else {
                shooter.setVelocity(0);
                shooter2.setVelocity(0);
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
            telemetry.addData("Target", shooterVelocity);
            telemetry.addData("Shooter1 Vel", shooter.getVelocity());
            telemetry.addData("Shooter2 Vel", shooter2.getVelocity());
            telemetry.addData("Shooter1 Err", shooterVelocity - shooter.getVelocity());
            telemetry.addData("Shooter2 Err", shooterVelocity - shooter2.getVelocity());

            telemetry.addData("rx", rx);
            telemetry.addData("Battery V", hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.addData("Loop Time (ms)", loopTimer.milliseconds());
            loopTimer.reset();
            telemetry.update();
        }
    }
}

