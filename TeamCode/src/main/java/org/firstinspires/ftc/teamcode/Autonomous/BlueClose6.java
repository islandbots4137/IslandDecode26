package org.firstinspires.ftc.teamcode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Constants.TeleOpConstants.*;

@Autonomous(name = "Blue Close 6", group = "Autonomous")
public class BlueClose6 extends LinearOpMode {

    // ---------- PATHING ----------
    private Follower follower;
    private Paths paths;
    private int pathState = 0;

    // ---------- HARDWARE ----------
    private DcMotorEx shooterMotor;

    private DcMotorEx shooterMotor2;
    private DcMotorEx magazine;
    private DcMotor roller;
    private Servo servo;
    DigitalChannel magLimitSwitch;

    private int positions = 0; // magazine index
    private ElapsedTime stateTimer = new ElapsedTime();
    public boolean actionStarted;

    @Override
    public void runOpMode() throws InterruptedException {


        // ---------------- HARDWARE MAP ----------------
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2=hardwareMap.get(DcMotorEx.class,"shooter2");
        magazine     = hardwareMap.get(DcMotorEx.class, "magazine");
        roller       = hardwareMap.get(DcMotor.class, "intake");
        servo  = hardwareMap.get(Servo.class, "servo");

        magazine.setTargetPosition(0);
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(77.4, 0, 0, 12.777);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        magLimitSwitch = hardwareMap.get(DigitalChannel.class, "magSwitch"); // your config name
        magLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setPosition(servoBack); // neutral

        // ---------- PATHING INIT ----------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(25.6, 129.87, Math.toRadians(144)));
        paths = new Paths(follower);

        telemetry.addLine("READY — Press PLAY");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // ---------------- AUTONOMOUS LOOP ----------------
        while (opModeIsActive() && pathState != -1) {
            follower.update();

            switch (pathState) {

                case 0:
                    // Start path once
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path1);
                        startShooterSlow();   // start shooter
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }

                    // After 2 second, feed the ball
                    if (actionStarted && stateTimer.seconds() > 2) {
                        feedOneBall();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 3) {
                        feedOneBall();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 4) {
                        feedOneBall();
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 5) {
                        feedOneBall();
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 6) {
                        stopShooter();
                        nextPathState();      // move to next state
                        actionStarted = false; // reset flag for next state
                    }
                    break;


                case 1:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path2);
                        rollerOn();
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();      // move to next state
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;

                case 2:
                    // Start path once
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path3);
                        rollerOn();
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();      // move to next state
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;

                case 3:
                    // Start path once
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path4);
                        rollerOn();
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();      // move to next state
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;

                case 4:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path5);
                        rollerOn();
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;

                case 5:
                    if (!follower.isBusy() && !actionStarted) {
                        startShooterSlow();
                        follower.followPath(paths.Path6);
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started

                    }
                    // After 2 second, feed the ball
                    if (actionStarted && stateTimer.seconds() > 1) {
                        feedOneBall();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 2) {
                        feedOneBall();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 3) {
                        feedOneBall();
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 4) {
                        feedOneBall();
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 5) {
                        stopShooter();
                        nextPathState();      // move to next state
                        actionStarted = false; // reset flag for next state
                    }
                    break;
            }

            // Telemetry
            telemetry.addData("Path State", pathState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

        // Final cleanup
        stopShooter();
        rollerOff();
        telemetry.addLine("Autonomous Complete");
        telemetry.update();
        sleep(2000);
    }

    private void nextPathState() {
        pathState++;
    }

    // ---------------- PATH CLASS ----------------

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(25.605, 129.866),

                                    new Pose(54.000, 89.250)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 89.250),

                                    new Pose(41.000, 83.750)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.000, 83.750),

                                    new Pose(36.000, 83.750)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(36.000, 83.750),

                                    new Pose(32.000, 83.750)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(32.000, 83.750),

                                    new Pose(24.000, 83.750)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(24.000, 83.750),
                                    new Pose(44.100, 84.310),
                                    new Pose(54.000, 89.250)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();
        }
    }


    // ---------------- HARDWARE HELPER FUNCTIONS ----------------

    public void rollerOn() { roller.setPower(-0.8); }
    public void rollerOff() { roller.setPower(0.0); }

    public void startShooterFast() { shooterMotor.setVelocity(1290); }
    public void startShooterSlow() {
        shooterMotor.setVelocity(1200);
        shooterMotor2.setVelocity(1200);}
    public void stopShooter() {
        shooterMotor.setVelocity(0);
        shooterMotor2.setVelocity(0);
    }
    public void feedOneBall() {
        // Fire servo
        servo.setPosition(servoForward);
        sleep(300);
        // Retract servo
        servo.setPosition(servoBack);
        sleep(300);

        // Fast encoder move toward next slot
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        magazine.setTargetPosition(-220);
        magazine.setPower(0.35);

        while (opModeIsActive() && magazine.isBusy()) {
            // wait for encoder move to finish
        }

        // Slow creep until magnet found
        magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        magazine.setPower(-0.12);

        while (opModeIsActive() && magLimitSwitch.getState()) {
            // wait for magnet
        }

        magazine.setPower(0);
        magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void spinUp() {
        // Fast encoder move
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setTargetPosition(-220);
        magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        magazine.setPower(-0.4);

        ElapsedTime stuckTimer = new ElapsedTime();
        int lastPos = magazine.getCurrentPosition();

        // Wait for encoder move, with stuck detection
        while (opModeIsActive() && magazine.isBusy()) {
            if (stuckTimer.milliseconds() > 500) {
                int currentPos = magazine.getCurrentPosition();
                if (Math.abs(currentPos - lastPos) < 5) {
                    // Stuck — reverse to free
                    magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    magazine.setPower(0.3);
                    servo.setPosition(servoBack);
                    sleep(200);

                    // Restart forward move
                    magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    magazine.setTargetPosition(-220);
                    magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    magazine.setPower(-0.4);
                }
                lastPos = magazine.getCurrentPosition();
                stuckTimer.reset();
            }
            idle();
        }

        // Slow creep until magnet found
        if (magLimitSwitch.getState()) {
            magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            magazine.setPower(-0.12);

            while (opModeIsActive() && magLimitSwitch.getState()) {
                idle();
            }
        }

        magazine.setPower(0);
        magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

