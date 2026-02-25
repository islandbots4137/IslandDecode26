package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red Close 6 Defunct", group = "Autonomous")
public class RedClose extends LinearOpMode {

    // ---------- PATHING ----------
    private Follower follower;
    private Paths paths;
    private int pathState = 0;

    // ---------- HARDWARE ----------
    private DcMotorEx shooterMotor;
    private DcMotorEx magazine;
    private DcMotor roller;
    private Servo feederServo;
    private int positions = 0; // magazine index
    private ElapsedTime stateTimer = new ElapsedTime();
    public boolean actionStarted;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- HARDWARE MAP ----------------
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        magazine     = hardwareMap.get(DcMotorEx.class, "magazine");
        roller       = hardwareMap.get(DcMotor.class, "intake");
        feederServo  = hardwareMap.get(Servo.class, "servo");

        magazine.setTargetPosition(0);
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        magazine.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        feederServo.setPosition(0.6); // neutral

        // ---------- PATHING INIT ----------
        follower = Constants.createFollower(hardwareMap);
        // mirror of (21, 124, 145°) -> (123, 124, 35°)
        follower.setStartingPose(new Pose(123, 124, Math.toRadians(35)));
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
                        follower.followPath(paths.line1);
                        startShooterFast();   // start shooter
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }

                    // After 1 second, feed the ball
                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        feedOneBall();
                    }
                    if (actionStarted && stateTimer.seconds() > 3) {
                        feedOneBall();
                    }
                    if (actionStarted && stateTimer.seconds() > 4) {
                        feedOneBall();
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
                   if (actionStarted && stateTimer.seconds() > 1) {
                        spinUp();
                        nextPathState();
                        actionStarted = false;
                    }  // save time?
                    break;

                case 4:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path5);
                        rollerOn();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 1) {
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
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path7);
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

                case 7:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path8);
                        nextPathState();
                    }
                    break;

                case 8:
                    if (!follower.isBusy() && !actionStarted) {
                        startShooterFast();
                        stateTimer.reset();
                        actionStarted = true;
                    }

                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        feedOneBall();
                    }
                    if (actionStarted && stateTimer.seconds() > 3.5) {
                        feedOneBall();
                    }
                    if (actionStarted && stateTimer.seconds() > 4.5) {
                        feedOneBall();
                    }
                    if (actionStarted && stateTimer.seconds() > 5.5) {
                        feedOneBall();
                    }
                    if (actionStarted && stateTimer.seconds() > 6.5) {
                        stopShooter();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;

                case 9:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path9);
                        nextPathState();
                    }
                    break;

                case 10:
                    if (!follower.isBusy() && !actionStarted) {
                        stopShooter();
                        pathState = -1; // finished
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

        public PathChain line1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

        public Paths(Follower follower) {

            // mirror x with x' = 144 - x, headings with θ' = π - θ

            line1 = follower.pathBuilder()
                    // (21,124) -> (50,102), 145° -> 143°
                    // → (123,124) -> (94,102), 35° -> 37°
                    .addPath(new BezierLine(
                            new Pose(123, 124),
                            new Pose(94, 102)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(35),
                            Math.toRadians(33))
                    .build();

            Path2 = follower.pathBuilder()
                    // (50,102),(91,95),(56,82), 143° -> 180°
                    // → (94,102),(53,95),(88,82), 37° -> 0°
                    .addPath(new BezierCurve(
                            new Pose(94, 102),
                            new Pose(53, 95),
                            new Pose(88, 84)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(33),
                            Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    // (56,82)->(48,82) → (88,82)->(96,82)
                    .addPath(new BezierLine(
                            new Pose(88, 84),
                            new Pose(100, 84)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    // (48,82)->(42,82) → (96,82)->(102,82)
                    .addPath(new BezierLine(
                            new Pose(100, 84),
                            new Pose(108, 84)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    // (42,82)->(36,82) → (102,82)->(108,82)
                    .addPath(new BezierLine(
                            new Pose(108, 84),
                            new Pose(116, 84)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder()
                    // (36,82)->(30,82) → (108,82)->(114,82)
                    .addPath(new BezierLine(
                            new Pose(116, 84),
                            new Pose(122, 84)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder()
                    // (36,82)->(30,82) → (108,82)->(114,82)
                    .addPath(new BezierLine(
                            new Pose(122, 84),
                            new Pose(128, 84)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            Path8 = follower.pathBuilder() // return to shoot
                    // (36,82),(89,83),(50,102) → (108,82),(55,83),(94,102)
                    // 180° -> 145° → 0° -> 35°
                    .addPath(new BezierCurve(
                            new Pose(108, 84),
                            new Pose(55, 83),
                            new Pose(94, 102)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(32.5))
                    .build();

            Path9 = follower.pathBuilder()
                    // (36,82)->(30,82) → (108,82)->(114,82)
                    .addPath(new BezierLine(
                            new Pose(94, 102),
                            new Pose(87, 102)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

//            Path6 = follower.pathBuilder()
//                    .addPath(new BezierCurve(new Pose(41, 103), new Pose(74.965, 78.761), new Pose(13, 70)))
//                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(90))
//                    .build();
        }
    }

    // ---------------- HARDWARE HELPER FUNCTIONS ----------------


    private void rollerOn() { roller.setPower(0.8); }
    private void rollerOff() { roller.setPower(0.0); }

    private void startShooterFast() { shooterMotor.setPower(-.3425); } // -0.3425
    private void startShooterSlow() { shooterMotor.setVelocity(-990); }
    private void stopShooter() { shooterMotor.setPower(0.0); }

    private void feedOneBall() throws InterruptedException {
        feederServo.setPosition(0);
        sleep(300);
        feederServo.setPosition(0.6);
        sleep(800); // working: 800 Try Next: 700

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