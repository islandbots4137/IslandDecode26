package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Constants.TeleOpConstants.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp
public class teleop2 extends LinearOpMode {
    private Limelight3A limelight3A;
    boolean lastPressed = false;
    int sequenceState = 0;
    boolean lastDpadUp = false;
    boolean indexing = false;
    boolean pushToggleState = false;
    boolean servoToggleState = false;
    boolean lastDpadUpG2 = false;
    boolean lastDpadDownG2 = false;
    boolean lastCrossG2 = false;
    double[] stepSizes = {5, 10, 50};
    int stepIndex = 2; // starts at 50

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
    boolean autoTurnEnabled = false;

    //DigitalChannel magLimitSwitch;
    boolean lastTriangleState = false;  // For edge detection
    boolean lastSquare = false;   // for edge detection


    @Override
    public void runOpMode() throws InterruptedException {
        ShooterRunning = false;
        ShooterRunningFast = false;

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        Servo aimLight = hardwareMap.get(Servo.class, "aimLight");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx magazine = hardwareMap.get(DcMotorEx.class, "magazine");
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients magazinePIDF = new PIDFCoefficients(10.0, 0, 0, 12.0);
        magazine.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, magazinePIDF);
        Servo servo = hardwareMap.servo.get("servo");

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(75.7, 0, 0, 10.577);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        // Reverse the left side motors.

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ElapsedTime timer = new ElapsedTime();
        GoBildaPinpointDriver odo;
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(0.2, -6.5, DistanceUnit.INCH);
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
            // Add this at the top of your loop to monitor
            telemetry.addData("Loop time", timer.milliseconds());
            timer.reset();
            double y = -gamepad1.left_stick_y; //front-back;  remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; //left-right
            double rx = gamepad1.right_stick_x;//rotation
            boolean runAutoTurn;
            if (gamepad2.cross && !lastCrossG2) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            lastCrossG2 = gamepad2.cross;

// ── Shooter speed adjustment ──
            double speedStep = stepSizes[stepIndex];

            if (gamepad2.dpad_up && !lastDpadUpG2) {
                shooterVelocity += speedStep;
                shooterVelocityFar += speedStep;
            }
            if (gamepad2.dpad_down && !lastDpadDownG2) {
                shooterVelocity -= speedStep;
                shooterVelocityFar -= speedStep;
            }
            lastDpadUpG2 = gamepad2.dpad_up;
            lastDpadDownG2 = gamepad2.dpad_down;



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

            double lightPos = NO_TAG_POS;

            if (gamepad2.right_bumper) {
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
            if (gamepad1.squareWasPressed()) {
                ShooterRunning = !ShooterRunning;
                if (ShooterRunning) ShooterRunningFast = false;
            }

            if (gamepad1.crossWasPressed()) {
                ShooterRunningFast = !ShooterRunningFast;
                if (ShooterRunningFast) ShooterRunning = false;
            }

            if (ShooterRunningFast) {
                shooter.setVelocity(-shooterVelocityFar);
                shooter2.setVelocity(-shooterVelocityFar);
            } else if (ShooterRunning) {
                shooter.setVelocity(shooterVelocity);
                shooter2.setVelocity(shooterVelocity);
            } else {
                shooter.setVelocity(0);
                shooter2.setVelocity(0);
            }

            if (gamepad1.triangleWasPressed()) {
                pushToggleState = !pushToggleState;
                servo.setPosition(servoLeave);
            }
            if (gamepad1.triangleWasReleased()) {
                pushToggleState = !pushToggleState;
                servo.setPosition(servoPush);
            }
            if (gamepad1.leftBumperWasPressed()) {
                intakeReverse = !intakeReverse;
                if (intakeReverse) {
                    intake.setPower(intakeOut);
                    magazine.setVelocity(-magazineVeloicty);
                } else {
                    intake.setPower(0);
                    magazine.setVelocity(0);

                }
            }
            if (gamepad1.rightBumperWasPressed()) {
                intakeOn = !intakeOn;
                if (intakeOn) {
                    intake.setPower(-1);
                    magazine.setVelocity(magazineVeloicty);
                } else {
                    intake.setPower(0);
                    magazine.setPower(0);

                }
            }
            telemetry.addData("Step Size", speedStep);
            telemetry.addData("Shooter Speed", shooterVelocity);
            telemetry.addData("Shooter Speed Far", shooterVelocityFar);
            telemetry.update();

        }
    }
}

