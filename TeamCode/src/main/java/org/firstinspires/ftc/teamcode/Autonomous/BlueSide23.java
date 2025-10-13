package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Side 23", group = "Autos")
public class BlueSide23 extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    // Poses from the GeneratedPaths snippet
    private final Pose p1 = new Pose(56.000, 8.000);
    private final Pose p2 = new Pose(48.000, 80.000);
    private final Pose p3 = new Pose(37.000, 80.000);
    private final Pose p4 = new Pose(33.000, 80.000);
    private final Pose p5 = new Pose(31.000, 80.000);
    private final Pose p6 = new Pose(40.000, 100.000);

    private PathChain line1, line2, line3, line4, line5;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // Build each path chain exactly as exported
        line1 = follower.pathBuilder()
                .addPath(new BezierLine(p1, p2))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        line2 = follower.pathBuilder()
                .addPath(new BezierLine(p2, p3))
                .setTangentHeadingInterpolation()
                .build();

        line3 = follower.pathBuilder()
                .addPath(new BezierLine(p3, p4))
                .setTangentHeadingInterpolation()
                .build();

        line4 = follower.pathBuilder()
                .addPath(new BezierLine(p4, p5))
                .setTangentHeadingInterpolation()
                .build();

        line5 = follower.pathBuilder()
                .addPath(new BezierLine(p5, p6))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        // Set the starting field pose (heading is handled by interpolation)
        follower.setStartingPose(p1);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(line1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(line2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(line3);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(line4);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(line5);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    setPathState(-1); // DONE
                }
                break;
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}
