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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp
public class teleopBlue extends LinearOpMode {
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
    double autoShooterVelocity = shooterVelocity;
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
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx magazine = hardwareMap.get(DcMotorEx.class, "magazine");
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //   PIDFCoefficients magazinePIDF =
        //     magazine.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, magazinePIDF);
        Servo servo = hardwareMap.servo.get("servo");
        servo.setPosition(servoClose);
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
            LLResult llResult = limelight3A.getLatestResult();
            // Add this at the top of your loop to monitor
            telemetry.addData("Loop time", timer.milliseconds());
            timer.reset();
            double y = -gamepad1.left_stick_y; //front-back;  remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; //left-right
            double rx = gamepad1.right_stick_x;//rotation
            boolean runAutoTurn;

         /*   if (gamepad2.cross && !lastCrossG2) {
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

*/
            int targetId = 20; // change to whatever tag ID you want
            LLResultTypes.FiducialResult target = null;
            List<LLResultTypes.FiducialResult> fiducials = null;
            double distanceInches = 0;
            boolean tagVisible = false;
            if (llResult != null && llResult.isValid()) {
                fiducials = llResult.getFiducialResults();

                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (f.getFiducialId() == targetId) {
                            target = f;
                            break;
                        }
                    }
                }

                if (target != null) {
                    double tx = target.getTargetPoseCameraSpace().getPosition().x;
                    double ty = target.getTargetPoseCameraSpace().getPosition().y;
                    double tz = target.getTargetPoseCameraSpace().getPosition().z;

                    distanceInches = Math.sqrt(tx * tx + ty * ty + tz * tz) * 39.3701;
                    autoShooterVelocity = getShooterVelocity(distanceInches);  // ← ADD THIS
                    tagVisible = true;                                          // ← ADD THIS

                    telemetry.addData("Tag ID", target.getFiducialId());
                    telemetry.addData("Distance (in)", "%.1f", distanceInches);
                }                }
            else {
                telemetry.addData("Tag " + targetId, "Not found");
                telemetry.addData("Limelight", "No result");
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
                    if (llResult != null && llResult.isValid()) {
                        fiducials = llResult.getFiducialResults();
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
         /*
            if (gamepad1.squareWasPressed()) {
                ShooterRunning = !ShooterRunning;
                if (ShooterRunning) ShooterRunningFast = false;
            }

            if (gamepad1.crossWasPressed()) {
                ShooterRunningFast = !ShooterRunningFast;
                if (ShooterRunningFast) ShooterRunning = false;
            }

            if (ShooterRunningFast) {
                shooter.setVelocity(shooterVelocityFar);
                shooter2.setVelocity(shooterVelocityFar);
            } else if (ShooterRunning) {
                shooter.setVelocity(shooterVelocity);
                shooter2.setVelocity(shooterVelocity);
            } else {
                shooter.setVelocity(0);
                shooter2.setVelocity(0);
            }
            */
                // Update auto velocity if tag is visible

// Shooter toggles
                if (gamepad1.squareWasPressed()) {
                    ShooterRunning = !ShooterRunning;
                    if (ShooterRunning) ShooterRunningFast = false;
                }
                if (gamepad1.crossWasPressed()) {
                    ShooterRunningFast = !ShooterRunningFast;
                    if (ShooterRunningFast) ShooterRunning = false;
                }

// Apply velocity — auto if tag visible, manual defaults if not
                if (ShooterRunningFast) {
                    double vel = tagVisible ? autoShooterVelocity : shooterVelocityFar;
                    shooter.setVelocity(vel);
                    shooter2.setVelocity(vel);
                } else if (ShooterRunning) {
                    double vel = tagVisible ? autoShooterVelocity : shooterVelocity;
                    shooter.setVelocity(vel);
                    shooter2.setVelocity(vel);
                } else {
                    shooter.setVelocity(0);
                    shooter2.setVelocity(0);
                }

                if (gamepad1.triangleWasPressed()) {
                    pushToggleState = !pushToggleState;
                    servo.setPosition(0.72);
                }
                if (gamepad1.triangleWasReleased()) {
                    pushToggleState = !pushToggleState;
                    servo.setPosition(0.16);
                }
                if (gamepad1.leftBumperWasPressed()) {
                    intakeReverse = !intakeReverse;
                    if (intakeReverse) {
                        intake.setPower(intakeOut);
                        magazine.setVelocity(-magazineVelocity);
                    } else {
                        intake.setPower(0);
                        magazine.setVelocity(0);

                    }
                }
                if (gamepad1.rightBumperWasPressed()) {
                    intakeOn = !intakeOn;
                    if (intakeOn) {
                        intake.setPower(-1);
                        magazine.setVelocity(magazineVelocity);
                    } else {
                        intake.setPower(0);
                        magazine.setVelocity(0);

                    }
                }
                telemetry.addData("Shooter Speed", shooterVelocity);
                telemetry.addData("Shooter Speed Far", shooterVelocityFar);
                telemetry.addData("Tag Visible", tagVisible);
                telemetry.addData("Active Velocity", tagVisible ? autoShooterVelocity :
                        (ShooterRunningFast ? shooterVelocityFar : shooterVelocity));

                telemetry.update();

            }
    }
        // Add as a method in your class
        public double getShooterVelocity ( double distanceInches)
        {
            double[] distances = {32.5, 46.2, 50.5, 57.4, 77.0, 85.0};
            double[] velocities = {1390, 1340, 1260, 1290, 1440, 1510};

            // Clamp to range
            if (distanceInches <= distances[0]) return velocities[0];
            if (distanceInches >= distances[distances.length - 1])
                return velocities[velocities.length - 1];

            // Find the two surrounding points and interpolate
            for (int i = 0; i < distances.length - 1; i++) {
                if (distanceInches <= distances[i + 1]) {
                    double t = (distanceInches - distances[i]) / (distances[i + 1] - distances[i]);
                    return velocities[i] + t * (velocities[i + 1] - velocities[i]);
                }
            }
            return velocities[velocities.length - 1];
        }
    }



