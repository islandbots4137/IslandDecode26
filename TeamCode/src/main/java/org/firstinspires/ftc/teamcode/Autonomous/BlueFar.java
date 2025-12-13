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

@Autonomous(name = "BlueFarWorking", group = "Autonomous")
public class BlueFar extends LinearOpMode {

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

        feederServo.setPosition(0.60); // neutral

        // ---------- PATHING INIT ----------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 7.5, Math.toRadians(90)));
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
                        feedOneBall();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 4.5) {
                        feedOneBall();      // move to next state
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 6.5) {
                        shootOne();
                        actionStarted = true; // reset flag for next state
                    }
                    if (actionStarted && stateTimer.seconds() > 7) {
                        stopShooter();
                        nextPathState();      // move to next state
                        actionStarted = false; // reset flag for next state
                    }
                    break;

                case 1:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path2);
                        nextPathState();
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
                        rollerOn();   // start shooter
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
                        rollerOn();   // start shooter
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    if (actionStarted && stateTimer.seconds() > 1.5) {
                        // move to next state
                        rollerOff();
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;


                case 5:   // now drive away on Path6
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path6);
                        nextPathState();
                    }
                    break;
                case 6:
                    if (!follower.isBusy() && !actionStarted) {
                        // at the end of Path5 now
                        startShooterFast();
                        stateTimer.reset();
                        actionStarted = true;
                    }

                    if (actionStarted && stateTimer.seconds() > 2) {
                        feedOneBall();
                    }
                    if (actionStarted && stateTimer.seconds() > 4.5) {
                        feedOneBall();
                    }
                    if (actionStarted && stateTimer.seconds() > 6.5) {
                        feedOneBall();
                    }
                    if (actionStarted && stateTimer.seconds() > 8) {
                        stopShooter();
                        actionStarted = false;
                        nextPathState();   // go to case 6 after shooting
                    }
                    break;

         /*       case 6:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path7);
                        // Example motor action: shoot one ball

                        nextPathState();
                    }
                    break;

                case 7:
                    if (!follower.isBusy()) {
                        rollerOn();
                        sleep(500);
                        follower.followPath(paths.Path8);



                        nextPathState();
                    }
                    break;

                case 8:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path9);
                        sleep(500);

                        nextPathState();
                    }
                    break;

                case 9:
                    if (!follower.isBusy()) {
                        stopShooter();
                        pathState = -1; // finished
                    }
                    break;

          */
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

        public PathChain line1, Path2, Path3, Path4, Path5, Path6; //Path7 Path8 Path9

        public Paths(Follower follower) {

            line1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56, 7.6), new Pose(62, 12)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(108.3))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(62, 12), new Pose(59.308, 35.585), new Pose(61, 35)))
                    .setLinearHeadingInterpolation(Math.toRadians(108.3), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61, 37), new Pose(54, 35)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(54, 37), new Pose(50, 35)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(50, 37), new Pose(42, 35)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(42, 35), new Pose(78.761, 48.633), new Pose(61, 11)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

         /*
            Path7 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(61, 11), new Pose(65.951, 63.578), new Pose(39, 59.5)))
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(39, 59.5), new Pose(32, 59.5)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(32, 59.5), new Pose(18, 54)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            */
        }
    }

    // ---------------- HARDWARE HELPER FUNCTIONS ----------------

    private void rollerOn() { roller.setPower(0.8); }
    private void rollerOff() { roller.setPower(0.0); }

    private void startShooterFast() { shooterMotor.setPower(-.445); }
    private void startShooterSlow() { shooterMotor.setVelocity(-990); }
    private void stopShooter() { shooterMotor.setPower(0.0); }
    private void shootOne()
    {
        feederServo.setPosition(0.0);
        sleep(300);
        feederServo.setPosition(0.60);
    }

    private void feedOneBall() throws InterruptedException {
        feederServo.setPosition(0.0);
        sleep(300);
        feederServo.setPosition(0.60);
        sleep(800);
        positions -= 250;
        magazine.setPower(0.45);
        magazine.setTargetPosition(positions);
        sleep(350);
    }
    private void spinUp() throws InterruptedException {
        magazine.setPower(0.45);
        positions -= 250;
        magazine.setTargetPosition(positions);
    }
}
