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
        //magLimitSwitch = hardwareMap.get(DigitalChannel.class, "magSwitch"); // your config name
        //magLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotor magazine = hardwareMap.dcMotor.get("magazine");
        magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Servo servo = hardwareMap.servo.get("servo");
        Servo block = hardwareMap.servo.get("block");

        ElapsedTime indexTimer = new ElapsedTime();


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
            boolean dpadUpPressed = gamepad1.dpad_up;
            int serva = 0;
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
                sequenceState = 0;  // Back to idle
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
                if (ShooterRunning) {
                    shooter.setVelocity(shooterVelocity); // ? Pid to control velocity
                    shooter2.setVelocity(shooterVelocity);
                } else {
                    shooter.setVelocity(0);
                    shooter2.setVelocity(0);
                }
            }


            if (gamepad1.right_stick_button) {
                ShooterRunning = !ShooterRunning;
                if (ShooterRunning) {
                    shooter.setVelocity(shooterVelocityFar); // ? Pid to control velocity
                    shooter2.setVelocity(shooterVelocityFar);
                } else {
                    shooter.setPower(0);
                }
            }
            if (gamepad1.circleWasPressed()) {
                pushToggleState = !pushToggleState;
                servo.setPosition(pushToggleState ? servoPush : servoLeave);
            }
            if (gamepad1.crossWasPressed()) {
                servoToggleState = !servoToggleState;
                block.setPosition(servoToggleState ? servoOpen : servoClose);
            }

            if (gamepad1.leftBumperWasPressed()) {
                intakeReverse = !intakeReverse;
                if (intakeReverse) {
                    intake.setPower(intakeOut);
                    magazine.setPower(-magazinePower);
                } else {
                    intake.setPower(0);
                    magazine.setPower(0);

                }
            }
            if (gamepad1.rightBumperWasPressed()) {
                intakeOn = !intakeOn;
                if (intakeOn) {
                    intake.setPower(-1);
                    magazine.setPower(-.8);
                } else {
                    intake.setPower(0);
                    magazine.setPower(0);

                }
            }
        }
    }
}

