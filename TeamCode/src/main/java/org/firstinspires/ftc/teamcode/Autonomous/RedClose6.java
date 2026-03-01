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

@Autonomous(name = "Red Close 6", group = "Autonomous")
public class RedClose6 extends LinearOpMode {

    // ---------- PATHING ----------
    private Follower follower;
    private Paths paths;
    private int pathState = 0;

    // ---------- HARDWARE ----------
    private DcMotorEx shooterMotor;

    private DcMotorEx shooterMotor2;
    private DcMotorEx magazine;
    private DcMotor roller;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo servo;
    boolean rollerArrived = false;
    private int positions = 0; // magazine index
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
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo.setPosition(0.16);

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------- PATHING INIT ----------
        // Mirrored start: x = 144 - 25.6 = 118.4, heading = 180 - 144 = 36
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(118.4, 129.87, Math.toRadians(36)));
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
                        follower.followPath(paths.Path1);
                        rollerOn();
                        startShooter();
                        stateTimer.reset();
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy() && stateTimer.seconds() > 0.5) {
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
                case 7:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path8);
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy()) {
                        pathState = -1;
                        actionStarted = false;
                    }
                    break;
            }

            // Telemetry
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
    // All coordinates mirrored: new_x = 144 - old_x, y unchanged
    // All headings mirrored:    new_heading = 180° - old_heading


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public Paths(Follower follower) {
            // Original: (25.605, 129.866) -> (61.5, 87), heading 144° -> 136.5°
            // Mirrored: (118.395, 129.866) -> (82.5, 87), heading 36° -> 43.5°
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(118.395, 129.866),
                                    new Pose(82.5, 87)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(43.5))
                    .build();

            // Original: (61.5, 87) -> ctrl(51.11, 82.318) -> (45.656, 83.75), heading 136.5° -> 180°
            // Mirrored: (82.5, 87) -> ctrl(92.89, 82.318) -> (98.344, 83.75), heading 43.5° -> 0°
            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(82.5, 87),
                                    new Pose(92.890, 82.318),
                                    new Pose(98.344, 83.750)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43.5), Math.toRadians(0))
                    .build();

            // Original: (45.656, 83.75) -> (26.155, 83.75), tangent heading
            // Mirrored: (98.344, 83.75) -> (117.845, 83.75), tangent heading
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(98.344, 83.750),
                                    new Pose(117.845, 83.750)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Original: (26.155, 83.75) -> ctrl(52.11, 83.582) -> (61.5, 87), heading 180° -> 136.5°
            // Mirrored: (117.845, 83.75) -> ctrl(91.89, 83.582) -> (82.5, 87), heading 0° -> 43.5°
            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(117.845, 83.750),
                                    new Pose(91.890, 83.582),
                                    new Pose(82.5, 87)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43.5))
                    .build();

            // Original: (61.5, 87) -> ctrl(49.217, 65.797) -> (48, 61), heading 136.5° -> 180°
            // Mirrored: (82.5, 87) -> ctrl(94.783, 65.797) -> (96, 61), heading 43.5° -> 0°
            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(82.5, 87),
                                    new Pose(94.783, 65.797),
                                    new Pose(96, 61)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43.5), Math.toRadians(0))
                    .build();

            // Original: (48, 61) -> (32, 61), tangent heading
            // Mirrored: (96, 61) -> (112, 61), tangent heading
            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96, 61),
                                    new Pose(112.000, 61)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Original: (32, 61) -> ctrl(47.095, 70.792) -> (61.5, 87), heading 180° -> 136.5°
            // Mirrored: (112, 61) -> ctrl(96.905, 70.792) -> (82.5, 87), heading 0° -> 43.5°
            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(112.000, 61),
                                    new Pose(96.905, 70.792),
                                    new Pose(82.5, 87)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43.5))
                    .build();

            // Original: (61.5, 87) -> (55.784, 113.441), heading 136.5° -> 225°
            // Mirrored: (82.5, 87) -> (88.216, 113.441), heading 43.5° -> -45°
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.5, 87),
                                    new Pose(88.216, 113.441)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43.5), Math.toRadians(-45))
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