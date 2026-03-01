
package org.firstinspires.ftc.teamcode.Autonomous;


import static org.firstinspires.ftc.teamcode.Constants.TeleOpConstants.magazineVelocity;
import static org.firstinspires.ftc.teamcode.Constants.TeleOpConstants.shooterVelocity;
import static org.firstinspires.ftc.teamcode.Constants.TeleOpConstants.shooterVelocityFar;

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

@Autonomous(name = "Blue Far 12 Ball", group = "Autonomous")
public class BlueFar12 extends LinearOpMode {

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
        servo   = hardwareMap.get(Servo.class, "servo");

        magazine.setTargetPosition(0);
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        servo.setPosition(0.445); // neutral

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
                        rollerOn();
                        startShooterFar();
                        follower.followPath(paths.Path1);
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    if (actionStarted && stateTimer.seconds() > 3.5) {
                        shotSequence();
                        nextPathState();
                        actionStarted = false; // reset flag for next state

                        break;
                    }
                case 1:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path2);
                        rollerOn();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        rollerOn();     // move to next state
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;


                case 3:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path3);
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy()) {
                        nextPathState();
                        actionStarted = false;
                    }
                    break;

                // --------- SHOOT SEQUENCE STATE (uses sleep like your example)
                case 4:
                    if (!follower.isBusy() && !actionStarted) {
                        startShooterFar();   // start shooter
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }

                    // After 1 second, feed the ball
                    if (actionStarted && stateTimer.seconds() > 3.5) {
                        shotSequence();
                        nextPathState();
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
                        rollerOn();     // move to next state
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
                        rollerOn();     // move to next state
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;

                case 7:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path6);
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy()) {
                        shotSequence();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;
                case 8:
                    if (!follower.isBusy() && !actionStarted) {
                        startShooterFar();   // start shooter
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    // After 1 second, feed the ball
                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        nextPathState();
                        actionStarted = false;
                    }

                    break;
                case 9:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path7);
                        rollerOn();
                        startShooterFar();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        shotSequence();
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;
                case 10:
                    if (!follower.isBusy() && !actionStarted) {
                        startShooterFar();   // start shooter
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    // After 1 second, feed the ball
                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        nextPathState();
                        actionStarted = false;
                    }
                    break;
                case 11:
                    if (!follower.isBusy() && !actionStarted) {
                        startShooterFar();   // start shooter
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    // After 1 second, feed the ball
                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        shotSequence();
                        nextPathState();
                        actionStarted = false;
                    }
                case 12:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path11);
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy()) {
                        pathState = -1; // end autonomous
                        actionStarted = false;
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
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;

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
            Path11 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(60, 15),
                            new Pose(60, 36)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(113)).build();

        }


        }


    // ---------------- HARDWARE HELPER FUNCTIONS (KEPT) ----------------

    public void rollerOn() {
        roller.setPower(-1);
        magazine.setVelocity(magazineVelocity);
    }
    public void rollerOff() {
        roller.setPower(0.0);
        magazine.setVelocity(0);
    }

    public void startShooter() {
        shooterMotor.setVelocity(shooterVelocity);
        shooterMotor2.setVelocity(shooterVelocity);
    }

    public void startShooterFar() {
        shooterMotor.setVelocity(shooterVelocityFar);
        shooterMotor2.setVelocity(shooterVelocityFar);
    }

    public void stopShooter() {
        shooterMotor.setVelocity(0);
        shooterMotor2.setVelocity(0);
    }
    public void shotSequence() {
        servo.setPosition(0.72);
        sleep(300);
        servo.setPosition(0.16);
        sleep(300);
        servo.setPosition(0.72);
        sleep(300);
        servo.setPosition(0.16);
        servo.setPosition(0.72);
        sleep(300);
        servo.setPosition(0.16);
        sleep(300);
        servo.setPosition(0.72);
        jiggle();
        sleep(2700);
        servo.setPosition(0.16);
        sleep(300);
    }
    private void jiggle() {
        for (int i = 0; i < 3; i++) {
            magazine.setVelocity(magazineVelocity);
            sleep(150);
            magazine.setVelocity(-magazineVelocity);
            sleep(150);
        }
        magazine.setVelocity(magazineVelocity);
    }


    


}
