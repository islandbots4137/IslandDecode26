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

@Autonomous(name = "RedFar Defunct", group = "Autonomous")
public class RedFar extends LinearOpMode {

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

        // Starting pose mirrored from Blue:
        // Blue: (72, 8, 90deg) -> Red mirror is the SAME because x=72 is on the mirror line
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

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
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path1);
                        startShooterFast();
                        // optional: rollerOff();
                        stateTimer.reset();
                        actionStarted = true;
                    }

                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 3) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 4) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 5) {
                        stopShooter();
                        nextPathState();
                        actionStarted = false;
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
                        spinUp();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;

                case 2:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path3);
                        rollerOn();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;

                case 3:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path4);
                        rollerOn();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;

                case 4:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path5);
                        rollerOn();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;

                case 5:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path6);
                        rollerOn();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;

                case 6:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path7);
                        nextPathState();
                    }
                    break;

                case 7:
                    if (!follower.isBusy() && !actionStarted) {
                        startShooterFast();
                        stateTimer.reset();
                        actionStarted = true;
                    }

                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 3.5) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 4.5) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 5.5) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 6) {
                        stopShooter();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;

                case 8:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path8);
                        rollerOn();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;

                case 9:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path9);
                        rollerOn();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        spinUp();
                        nextPathState();
                        actionStarted = false;
                    }
                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        spinUp();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;

                case 10:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path10);
                        nextPathState();
                    }
                    // (your original code falls through; left unchanged on purpose)

                case 11:
                    if (!follower.isBusy() && !actionStarted) {
                        startShooterFast();
                        stateTimer.reset();
                        actionStarted = true;
                    }

                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 3.5) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 4.5) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 5.5) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 6) {
                        stopShooter();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;
            }

            telemetry.addData("Path State", pathState);
            telemetry.addData("Busy", follower.isBusy());
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

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

            // Mirroring used:
            // x' = 144 - x
            // heading' = 180deg - heading (normalized)

            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(88.489, 7.374),
                            new Pose(83.368, 8.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(72.2)).build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(83.368, 8.000),
                            new Pose(84.000, 44.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(73), Math.toRadians(0)).build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(84.000, 44.000),
                            new Pose(85.500, 44.000)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            Path4 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(85.500, 44.000),
                            new Pose(89.500, 44.000)
                    )
            ).setTangentHeadingInterpolation().build();

            Path5 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(89.500, 44.000),
                            new Pose(93.500, 44.000)
                    )
            ).setTangentHeadingInterpolation().build();

            Path6 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(93.500, 42.000),
                            new Pose(97.500, 42.000)
                    )
            ).setTangentHeadingInterpolation().build();

            Path7 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(97.500, 44.000),
                            new Pose(79.308, 8.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69)).build();

            Path8 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(79.308, 9.400),
                            new Pose(131.873, 12.582)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(315)).build();

            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(131.873, 12.582),
                            new Pose(133.996, 8.051)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(340)).build();

            Path10 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(133.996, 8.051),
                            new Pose(81.989, 19.942)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(69)).build();
        }
    }

    // ---------------- HARDWARE HELPER FUNCTIONS (KEPT) ----------------

    private void rollerOn()  { roller.setPower(-0.8); }
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
        magazine.setPower(0.45);
        positions -= 250;
        magazine.setTargetPosition(positions);
    }
}
