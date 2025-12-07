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

@Autonomous(name = "RedFarWorking", group = "Autonomous")
public class RedFar extends LinearOpMode {

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
        // Mirror of (56, 7.5, 90°) is (144-56=88, 7.5, 90°)
        follower.setStartingPose(new Pose(88, 7.5, Math.toRadians(90)));
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
                        follower.followPath(paths.line1);
                        startShooterFast();
                        stateTimer.reset();
                        actionStarted = true;
                    }

                    if (actionStarted && stateTimer.seconds() > 2.5) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 4.5) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 6.5) {
                        shootOne();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 7) {
                        stopShooter();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;

                case 1:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path2);
                        nextPathState();
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
                        rollerOff();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;

                case 5:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path6);
                        stateTimer.reset();
                        actionStarted = true;
                    }

                    if (actionStarted && stateTimer.seconds() > 2) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 4) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 6) {
                        feedOneBall();
                        actionStarted = true;
                    }
                    if (actionStarted && stateTimer.seconds() > 7) {
                        stopShooter();
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
            }

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

            // Mirrored paths:
            // x' = 144 - x, θ' = 180° - θ

            line1 = follower.pathBuilder()
                    // (56,7.6)->(61,12), 90°->110.5°  ->  (88,7.6)->(83,12), 90°->69.5°
                    .addPath(new BezierLine(
                            new Pose(88, 7.6),
                            new Pose(83, 12)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(69.5))
                    .build();

            Path2 = follower.pathBuilder()
                    // (61,12),(59.308,35.585),(56,36)  110.5°->180°
                    // -> (83,12),(84.692,35.585),(88,36) 69.5°->0°
                    .addPath(new BezierCurve(
                            new Pose(83, 12),
                            new Pose(84.692, 35.585),
                            new Pose(88, 36)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(69.5),
                            Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    // (56,36)->(52,36) -> (88,36)->(92,36)
                    .addPath(new BezierLine(
                            new Pose(88, 36),
                            new Pose(92, 36)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    // (52,36)->(44,36) -> (92,36)->(100,36)
                    .addPath(new BezierLine(
                            new Pose(92, 36),
                            new Pose(100, 36)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    // (44,36)->(39,36) -> (100,36)->(105,36)
                    .addPath(new BezierLine(
                            new Pose(100, 36),
                            new Pose(105, 36)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder()
                    // (39,36),(78.761,48.633),(61,11)  180°->110°
                    // -> (105,36),(65.239,48.633),(83,11) 0°->70°
                    .addPath(new BezierCurve(
                            new Pose(105, 36),
                            new Pose(65.239, 48.633),
                            new Pose(83, 11)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(70))
                    .build();

            Path7 = follower.pathBuilder()
                    // (61,11),(65.951,63.578),(39,59.5) 110°->180°
                    // -> (83,11),(78.049,63.578),(105,59.5) 70°->0°
                    .addPath(new BezierCurve(
                            new Pose(83, 11),
                            new Pose(78.049, 63.578),
                            new Pose(105, 59.5)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(70),
                            Math.toRadians(0))
                    .build();

            Path8 = follower.pathBuilder()
                    // (39,59.5)->(32,59.5) -> (105,59.5)->(112,59.5)
                    .addPath(new BezierLine(
                            new Pose(105, 59.5),
                            new Pose(112, 59.5)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            Path9 = follower.pathBuilder()
                    // (32,59.5)->(18,54) -> (112,59.5)->(126,54)
                    .addPath(new BezierLine(
                            new Pose(112, 59.5),
                            new Pose(126, 54)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();
        }
    }

    // ---------------- HARDWARE HELPER FUNCTIONS ----------------

    private void rollerOn() { roller.setPower(0.8); }
    private void rollerOff() { roller.setPower(0.0); }

    private void startShooterFast() { shooterMotor.setPower(-.46); }
    private void startShooterSlow() { shooterMotor.setVelocity(-990); }
    private void stopShooter() { shooterMotor.setPower(0.0); }

    private void shootOne() {
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
        magazine.setPower(0.7);
        magazine.setTargetPosition(positions);
        sleep(350);
    }

    private void spinUp() throws InterruptedException {
        magazine.setPower(0.7);
        positions -= 250;
        magazine.setTargetPosition(positions);
    }
}
