
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

@Autonomous(name = "Blue Close 12", group = "Autonomous")
public class BlueClose12 extends LinearOpMode {

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

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------- PATHING INIT ----------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(25.6, 129.87, Math.toRadians(144)));
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
                    // Start path once
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path1);
                        rollerOn();
                        startShooter();   // start shooter
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    if (actionStarted && stateTimer.seconds() > 2) {
                        shotSequence();
                         nextPathState();
                        actionStarted = false; // reset flag for next state
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
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    if (actionStarted && stateTimer.seconds() > 3.5) {
                        shotSequence();
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;
                case 4:
                    if (!follower.isBusy() && !actionStarted) {
                        rollerOn();
                        follower.followPath(paths.Path5);
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy()) {
                        nextPathState();
                        actionStarted = false;
                    }
                    break;
                case 5:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path6);
                        actionStarted = true;
                    }
                    if (actionStarted && !follower.isBusy()) {
                        nextPathState();
                        actionStarted = false;
                    }
                    break;
                    case 6:
                    if (!follower.isBusy() && !actionStarted) {
                        rollerOn();
                        follower.followPath(paths.Path7);
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    if (actionStarted && stateTimer.seconds() > 3.5) {
                        shotSequence();
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;

                case 7:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path8);
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    if (actionStarted && stateTimer.seconds() > 3.5) {
                        rollerOn();
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;
                case 8:
                    if (!follower.isBusy() && !actionStarted) {
                        rollerOn();
                        follower.followPath(paths.Path9);
                        stateTimer.reset();   // start 1-second delay
                        actionStarted = true; // mark that action has started
                    }
                    if (actionStarted && stateTimer.seconds() > 3.5) {
                        shotSequence();
                        nextPathState();
                        actionStarted = false; // reset flag for next state
                    }
                    break;
                case 9:
                    if (!follower.isBusy() && !actionStarted) {
                        follower.followPath(paths.Path10);
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
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(25.605, 129.866),

                                    new Pose(54.000, 89.250)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(54.000, 89.250),
                                    new Pose(51.110, 82.318),
                                    new Pose(41.656, 83.750)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.656, 83.750),

                                    new Pose(19.155, 83.750)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(19.155, 83.750),
                                    new Pose(52.110, 83.582),
                                    new Pose(54.000, 89.250)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(54.000, 89.250),
                                    new Pose(47.217, 65.797),
                                    new Pose(42.456, 59.982)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.456, 59.982),

                                    new Pose(20.000, 59.800)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(20.000, 59.800),
                                    new Pose(47.095, 70.792),
                                    new Pose(54.000, 89.250)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(54.000, 89.250),
                                    new Pose(47.119, 62.615),
                                    new Pose(13.300, 57.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.300, 60.000),
                                    new Pose(45.148, 64.991),
                                    new Pose(54.000, 89.250)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 89.250),

                                    new Pose(55.784, 113.441)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(225))

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
        // Drop servo to let balls through
        servo.setPosition(0.72);
        sleep(1000);

        // Reverse roller and magazine briefly to clear any jams
        roller.setPower(0.5);
        magazine.setVelocity(-magazineVelocity);
        sleep(300);
        // Spin forward to shoot
        roller.setPower(-1);
        magazine.setVelocity(magazineVelocity);
        // Drop servo once and bring back up
        servo.setPosition(0.16);
        sleep(300);
        servo.setPosition(0.72);
        sleep(1000);

        // Reset servo
        servo.setPosition(0.16);
    }

}


