package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Side 22 ", group = "Autos")
public class BlueSide22 extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    // Key poses from the generated routine
    private final Pose START_POS = new Pose(84.000, 8.000);     // Heading set below to 90°
    private final Pose C1       = new Pose(85.000, 30.000);     // Bezier control for Path 1
    private final Pose P1_END   = new Pose(96.000, 36.000);     // End of Path 1 / start of Path 2
    private final Pose P2_END   = new Pose(108.000, 36.000);    // End of Path 2 / start of Path 3
    private final Pose P3_END   = new Pose(114.000, 36.000);    // End of Path 3 / start of Path 4
    private final Pose P4_END   = new Pose(120.000, 36.000);    // End of Path 4 / start of Path 5
    private final Pose C2       = new Pose(87.000, 60.000);     // Bezier control for Path 5
    private final Pose END_POS  = new Pose(100.000, 100.000);   // End of Path 5

    private PathChain fullChain;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // Build the entire generated chain (combined)
        fullChain = follower.pathBuilder()
                // Path 1: BezierCurve (start -> control -> P1_END)
                .addPath(new BezierCurve(
                        new Pose(START_POS.getX(), START_POS.getY()),
                        new Pose(C1.getX(), C1.getY()),
                        new Pose(P1_END.getX(), P1_END.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                // Path 2: Line (P1_END -> P2_END)
                .addPath(new BezierLine(
                        new Pose(P1_END.getX(), P1_END.getY()),
                        new Pose(P2_END.getX(), P2_END.getY())
                ))
                .setTangentHeadingInterpolation()

                // Path 3: Line (P2_END -> P3_END)
                .addPath(new BezierLine(
                        new Pose(P2_END.getX(), P2_END.getY()),
                        new Pose(P3_END.getX(), P3_END.getY())
                ))
                .setTangentHeadingInterpolation()

                // Path 4: Line (P3_END -> P4_END)
                .addPath(new BezierLine(
                        new Pose(P3_END.getX(), P3_END.getY()),
                        new Pose(P4_END.getX(), P4_END.getY())
                ))
                .setTangentHeadingInterpolation()

                // Path 5: BezierCurve (P4_END -> C2 -> END_POS)
                .addPath(new BezierCurve(
                        new Pose(P4_END.getX(), P4_END.getY()),
                        new Pose(C2.getX(), C2.getY()),
                        new Pose(END_POS.getX(), END_POS.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        // Set starting pose. If your Pose class supports heading in the constructor, prefer:
        //    follower.setStartingPose(new Pose(84.0, 8.0, Math.toRadians(90)));
        // Otherwise this sets position; heading will be handled by the first interpolation.
        follower.setStartingPose(START_POS);
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
                // Kick off the single combined chain
                follower.followPath(fullChain);
                setPathState(1);
                break;

            case 1:
                // Wait until chain completes
                if (!follower.isBusy()) {
                    setPathState(-1); // DONE
                }
                break;

            // -1 == DONE (no default needed)
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}
