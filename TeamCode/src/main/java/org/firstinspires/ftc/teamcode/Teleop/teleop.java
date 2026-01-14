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

@TeleOp
public class teleop extends LinearOpMode {
    private Limelight3A limelight3A;
    boolean lastPressed = false;
    boolean indexing = false;
    boolean requireRelease = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors

    //    double F=12.777; // RIGHT=12.777;
      //  double P=77.4; // RIGHT= 77.4
       // PIDFCoefficients pidfCoefficients=new PIDFCoefficients(P,0,0,F);
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        Servo aimLight = hardwareMap.get(Servo.class, "aimLight");

        DigitalChannel magSwitch  = hardwareMap.get(DigitalChannel.class, "magHome");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor magazine = hardwareMap.dcMotor.get("magazine");
        Servo servo = hardwareMap.servo.get("servo");
        magazine.setTargetPosition(0);
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ElapsedTime indexTimer = new ElapsedTime();

        magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Reverse the left side motors.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        limelight3A.start();
        limelight3A.pipelineSwitch(8);
     //   boolean homePressed = !magHome.getState();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            telemetry.addData("magHome raw", magHome.getState());

            double y = -gamepad1.left_stick_y; //front-back;  remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; //left-right
            double rx = gamepad1.right_stick_x;//rotation
            boolean autoTurn=gamepad2.square;
            if (autoTurn) {
                LLResult llResult = limelight3A.getLatestResult();

                if (llResult != null && llResult.isValid()) {

                    List<LLResultTypes.FiducialResult> fiducials =
                            llResult.getFiducialResults();
                    telemetry.addData("rx", rx);

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
            };


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
            // Automated shooting
            if (gamepad1.dpad_up) {
                magazine.setPower(magazinePower);
                positions -= 250;                      // spin to next ball
                servo.setPosition(servoForward);
                sleep(300);
                servo.setPosition(servoBack);
                sleep(400);
                magazine.setTargetPosition(positions);

            }


            double lightPos = NO_TAG_POS;
            if(gamepad2.right_bumper) {
                LLResult llResult = limelight3A.getLatestResult();
                if (llResult != null && llResult.isValid()) {

                    List<LLResultTypes.FiducialResult> fiducials =
                            llResult.getFiducialResults();

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
            // PWM should be green when towards
           /*
            if (gamepad1.crossWasPressed()) {

                autoIntake = !autoIntake;
                spun = false;
            }
            // Commented out for now test how it works    spun = false;

            if (autoIntake) {
                if (detected == TestBenchColor.DetectedColor.TAN) {
                    intake.setPower(0.8);
                    spun = false;
                }
                if (detected == TestBenchColor.DetectedColor.UNKNOWN && !spun) {
                    magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    magazine.setPower(0.3);
                    positions = positions - 250;
                    magazine.setTargetPosition(positions);
                    spun = true;
                }
            } else {
                intake.setPower(0);
            }
            */

//                    intake.setPower(0);
//                }

            if (gamepad2.triangleWasPressed()) {
                magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                magazine.setPower(magazinePower);
                positions+=10;
                magazine.setTargetPosition(positions);
            }
            if (gamepad2.circleWasPressed()) {
                magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                magazine.setPower(magazinePower);
                positions-=10;
                magazine.setTargetPosition(positions);

            }
            if (gamepad1.squareWasPressed()) {
                ShooterRunning = !ShooterRunning;
                if (ShooterRunning) {
                    shooter.setVelocity(shooterVelocity); // ? Pid to control velocity
                    shooter2.setVelocity(shooterVelocity);
                } else {
                    shooter.setVelocity(0);
                }
            }
            // Just edit positions array once obtained
           /*
            if (gamepad1.triangleWasPressed()) {

                    // spin slowly until switch hits
                    magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    magazine.setPower(0.2); // slow
                positions = positions - 250;
                magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                magazine.setTargetPosition(positions);
                magazine.setPower(magazinePower);

            } */
            boolean pressed = !magSwitch.getState();      // if backwards, remove the '!'
            boolean pressedEdge = pressed && !lastPressed;
            lastPressed = pressed;

// Start indexing on triangle press (only if not currently indexing)
            if (gamepad1.triangleWasPressed() && !indexing && !requireRelease) {
                indexing = true;
                indexTimer.reset();

                magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                magazine.setPower(INDEX_POWER);
            }

// While indexing, stop when switch triggers
            if (indexing) {
                if (pressedEdge) {
                    magazine.setPower(0);
                    indexing = false;
                    requireRelease = true;  // wait until magnet leaves sensor
                } else if (indexTimer.seconds() > INDEX_TIMEOUT_S) {
                    // failsafe if switch never triggers
                    magazine.setPower(0);
                    indexing = false;
                    requireRelease = false;
                }
            }

// Allow another index once switch is released
            if (!pressed) {
                requireRelease = false;
            }

            telemetry.addData("MagSwitch pressed", pressed);
            telemetry.addData("Indexing", indexing);

            if (gamepad1.dpadLeftWasPressed()) {

                positions = positions + 250;

                magazine.setTargetPosition(positions);
                magazine.setPower(magazinePower);
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
                servo.setPosition(servoForward);
                sleep(200);
                servo.setPosition(servoBack);
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
            telemetry.addData("y x rx", "%.3f %.3f %.3f", y, x, rx);
            telemetry.addData("FL FR", "%.3f %.3f", frontLeftPower, frontRightPower);
            telemetry.addData("BL BR", "%.3f %.3f", backLeftPower, backRightPower);
        }
    }
}

