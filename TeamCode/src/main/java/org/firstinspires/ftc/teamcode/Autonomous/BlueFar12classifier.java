package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Far 12 Classifier", group = "Autonomous")
public class BlueFar12classifier extends LinearOpMode {

    // ---------- PATHING ----------
    private Follower follower;
    private Paths paths;
    private int pathState = 0;

    // ---------- HARDWARE ----------
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private DcMotorEx magazine;
    private DcMotor roller;
    private Servo feederServo;

    // magazine indexing
    private int positions = 0;
    boolean magazineRotating = false;

    // state helpers
    private ElapsedTime stateTimer = new ElapsedTime();
    public boolean actionStarted = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- HARDWARE MAP ----------------
        shooterMotor  = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        magazine      = hardwareMap.get(DcMotorEx.class, "magazine");
        roller        = hardwareMap.get(DcMotor.class, "intake");
        feederServo   = hardwareMap.get(Servo.class, "servo");

        magazine.setTargetPosition(0);
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        feederServo.setPosition(0.6); // neutral

        // ---------- PATHING INIT ----------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        paths = new Paths(follower);

        telemetry.addLine("READY — Press PLAY");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // ---------------- AUTONOMOUS LOOP ----------------
        while (opModeIsActive() && pathState != -1) {
            follower.update();

            switch (pathState) {

                // --------- Example flow: Path1 -> Path2 -> ... -> Path9
                // Adjust which states shoot/intake based on your routine.

                case 0:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path1);
                         optional: startShooterFast();
                        // optional: rollerOff();
                        stateTimer.reset();
                        actionStarted = true;
                    }

                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        spinUp();     // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 3) {
                        push();     // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 3.5) {
                        spinUp();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 4) {
                        push();
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 4.5) {
                        spinUp();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 5) {
                        push();
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 5.5) {
                        stopShooter();
                        nextPathState();      // move to next state
                        actionStarted = false; // reset flag for next state
                    }
                    break;

                case 1:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path2);
                       rollerOn();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();      // move to next state
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;


                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path3);
                        nextPathState();
                    }
                    break;

                // --------- SHOOT SEQUENCE STATE (uses sleep like your example)
                case 4:
                    if (!follower.isBusy() && !actionStarted) {
                        startShooterFast();   // start shooter
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }

                    // After 1 second, feed the ball
                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        feedOneBall();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 3.5) {
                        feedOneBall();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 4.5) {
                        feedOneBall();
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 5.5) {
                        feedOneBall();
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 6) {
                        stopShooter();
                        nextPathState();      // move to next state
                        actionStarted = false; // reset flag for next state
                    }
                    break;
                case 5:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path4);
                        rollerOn();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;

                case 6:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path5);
                        rollerOn();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        spinUp();
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;

                case 7:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path6);
                        nextPathState();
                    }
                case 8:
                    if (!follower.isBusy() && !actionStarted) {
                        startShooterFast();   // start shooter
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }

                    // After 1 second, feed the ball
                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        feedOneBall();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 3.5) {
                        feedOneBall();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 4.5) {
                        feedOneBall();
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 5.5) {
                        feedOneBall();
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 6) {
                        stopShooter();
                        nextPathState();      // move to next state
                        actionStarted = false; // reset flag for next state
                    }

                    break;
                case 9:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path7);
                        rollerOn();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;
                case 10:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path8);
                        nextPathState();
                    }
                case 11:
                    if (!follower.isBusy() && !actionStarted) {
                        startShooterFast();   // start shooter
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }

                    // After 1 second, feed the ball
                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        feedOneBall();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 3.5) {
                        feedOneBall();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 4.5) {
                        feedOneBall();
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 5.5) {
                        feedOneBall();
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 6) {
                        stopShooter();
                        nextPathState();      // move to next state
                        actionStarted = false; // reset flag for next state
                    }

                    break;
            }

            // Telemetry
            telemetry.addData("Path State", pathState);
            telemetry.addData("Busy", follower.isBusy());
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
        sleep(1000);
    }

    private void nextPathState() {
        pathState++;
    }

    // ---------------- PATH CLASS ----------------
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56, 8),
                            new Pose(60, 15)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(111)).build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(60, 15),
                            new Pose(48, 35.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(111), Math.toRadians(180)).build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(48, 35.000),
                            new Pose(20, 35.000)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path4 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(20.00, 35.000),
                            new Pose(60, 15)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113)).build();
            //shoot second 3 balls
            Path5 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(60, 15),
                            new Pose(48, 60)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180)).build();

            Path6 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(48, 60),
                            new Pose(22, 60)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Path7 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(22, 60.000),
                            new Pose(60, 15)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113)).build();
            //shoot again
            Path8 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(60, 15),
                            new Pose(8, 27)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(210)).build();

            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(8, 27),
                            new Pose(8, 6)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(210)).build();

            Path10 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(8, 6),
                            new Pose(60, 15)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(113)).build();

        }


        }


    // ---------------- HARDWARE HELPER FUNCTIONS (KEPT) ----------------

    private void rollerOn()  { roller.setPower(-1); }
    private void rollerOff() { roller.setPower(0.0); }

    private void startShooterFast() {
        shooterMotor.setVelocity(1300);
        shooterMotor2.setVelocity(1300);
    }

    private void startShooterSlow() {
        shooterMotor.setVelocity(1280);
        shooterMotor2.setVelocity(1280);
    }

    private void stopShooter() {
        shooterMotor.setPower(0.0);
        shooterMotor2.setPower(0.0);
    }
    private void push() {
        feederServo.setPosition(0.1);
        sleep(300);
        feederServo.setPosition(0.6);
    }
    private void feedOneBall() throws InterruptedException {
        feederServo.setPosition(0.1);
        sleep(300);
        feederServo.setPosition(0.6);
        sleep(800);
        positions -= 250;
        magazine.setPower(0.45);
        magazine.setTargetPosition(positions);
        sleep(400);
    }

    private void spinUp() throws InterruptedException {
        magazineRotating = true;
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setTargetPosition(-230); // tune this — slightly SHORT of the magnet
        magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        magazine.setPower(-0.35);
    }
}
