
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
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Constants.TeleOpConstants.*;

@Autonomous(name = "Blue Far 9", group = "Autonomous")
public class BlueFar9 extends LinearOpMode {

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
    boolean rollerArrived = false;
    private ElapsedTime stateTimer = new ElapsedTime();
    public boolean actionStarted;

    @Override
    public void runOpMode() throws InterruptedException {


        // ---------------- HARDWARE MAP ----------------
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2=hardwareMap.get(DcMotorEx.class,"shooter2");
        magazine = hardwareMap.get(DcMotorEx.class, "magazine");
        roller = hardwareMap.get(DcMotor.class, "intake");
        servo  = hardwareMap.get(Servo.class, "servo");
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  PIDFCoefficients shooterPIDF = new PIDFCoefficients(75.7, 0, 0, 10.577);
        //  shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        //  shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo.setPosition(0.16);

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------- PATHING INIT ----------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        paths = new Paths(follower);

        telemetry.addLine("READY — Press PLAY");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        //AUTO LOOP
        while (opModeIsActive() && pathState != -1) {
            follower.update();

            switch (pathState) {

                case 0:
                    if (!follower.isBusy() && !actionStarted) {
                        rollerOn();
                        startShooter();
                        follower.followPath(paths.Path1);
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy() && stateTimer.seconds() > 1) {
                        shotSequence();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;

                case 1:
                    if (!follower.isBusy() && !actionStarted) {
                        rollerOn();
                        follower.followPath(paths.Path2);
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy()) {
                        nextPathState();
                        actionStarted = false;
                    }
                    break;
                case 2:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path3);
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy()) {
                        nextPathState();
                        actionStarted = false;
                    }
                    break;
                case 3:
                    if (!follower.isBusy() && !actionStarted) {
                        rollerOn();
                        follower.followPath(paths.Path4);
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy() && stateTimer.seconds() > 2) {
                        shotSequence();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;
                case 4:
                    if (!follower.isBusy() && !actionStarted) {
                        rollerOn();
                        follower.followPath(paths.Path5);
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy() && stateTimer.seconds()>2)
                    {
                        nextPathState();
                        actionStarted = false;
                    }
                    break;
                case 5:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path6);
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy() && stateTimer.seconds() > 2)
                    {
                        shotSequence();
                        nextPathState();
                        actionStarted = false;
                    }
                    break;
                case 6:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path7);
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy())
                    {
                        pathState = -1;
                        actionStarted = false;
                    }
                    break;

            }
            telemetry.addData("Path State", pathState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Shooter 1 Vel", shooterMotor.getVelocity());
            telemetry.addData("Shooter 2 Vel", shooterMotor2.getVelocity());
            telemetry.addData("Shooter Target", shooterVelocity);
            telemetry.addData("Magazine Vel", magazine.getVelocity());
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
        public PathChain Path3;
        public PathChain Path2;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;


        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(57.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.000, 12.000),
                                    new Pose(53.851, 25.406),
                                    new Pose(46.333, 34.628)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.333, 34.628),

                                    new Pose(20.575, 34.848)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.575, 34.848),

                                    new Pose(57.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 12.000),

                                    new Pose(14.593, 10.406)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(190))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.593, 10.406),

                                    new Pose(57.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(190), Math.toRadians(110))

                    .build();
            Path7 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(57.000, 12.000),

                            new Pose(36.059, 13.125)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();
        }
    }



    // ---------------- HARDWARE HELPER FUNCTIONS ----------------

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

