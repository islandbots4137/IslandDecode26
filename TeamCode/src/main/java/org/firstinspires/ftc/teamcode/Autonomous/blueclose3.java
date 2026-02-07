package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

public class blueclose3 {

    @Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
    @Configurable
    public class PedroAutonomous extends OpMode {

        private TelemetryManager panelsTelemetry;
        public Follower follower;
        private int pathState;
        private Paths paths;

        @Override
        public void init() {
            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

            paths = new Paths(follower);

            pathState = 0;

            panelsTelemetry.debug("Status", "Initialized");
            panelsTelemetry.update(telemetry);
        }

        @Override
        public void loop() {
            follower.update();

            autonomousPathUpdate();

            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }

        public void autonomousPathUpdate() {
            switch (pathState) {

                case 0:
                    follow(paths.Path1);
                    break;

                case 1:
                    follow(paths.Path2);
                    break;

                case 2:
                    follow(paths.Path3);
                    break;

                case 3:
                    follow(paths.Path4);
                    break;

                case 4:
                    follow(paths.Path5);
                    break;

                case 5:
                    follow(paths.Path6);
                    break;

                case 6:
                    follow(paths.Path7);
                    break;

                case 7:
                    follow(paths.Path8);
                    break;

                case 8:
                    follow(paths.Path9);
                    break;

                case 9:
                    follow(paths.Path10);
                    break;

                case 10:
                    follow(paths.Path11);
                    break;

                case 11:
                    follow(paths.Path12);
                    break;

                default:
                    // done
                    break;
            }
        }

        /* ================= HELPER METHODS ================= */

        private void follow(PathChain path) {
            if (!follower.isBusy()) {
                follower.followPath(path);
                pathState++;
            }
        }

        private boolean isFinished() {
            return !follower.isBusy();
        }

        /* ================= PATH DEFINITIONS ================= */

        public class Paths {
            public PathChain Path1, Path2, Path3, Path4, Path5, Path6;
            public PathChain Path7, Path8, Path9, Path10, Path11, Path12;

            public Paths(Follower follower) {

                Path1 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(56.000, 8.000),
                                        new Pose(60.000, 14.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(108))
                        .build();

                Path2 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(60.000, 14.000),
                                        new Pose(64.464, 37.525),
                                        new Pose(48.000, 36.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(108), Math.toRadians(180))
                        .build();

                Path3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(48.000, 36.000),
                                        new Pose(39.425, 36.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                Path4 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(39.425, 36.000),
                                        new Pose(34.000, 36.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                Path5 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(34.000, 36.000),
                                        new Pose(27.000, 36.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                Path6 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(27.000, 36.000),
                                        new Pose(67.450, 38.790),
                                        new Pose(57.227, 10.674)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(108))
                        .build();

                Path7 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(57.227, 10.674),
                                        new Pose(37.127, 32.354),
                                        new Pose(27.050, 25.193)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(108), Math.toRadians(180))
                        .build();

                Path8 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(27.050, 25.193),
                                        new Pose(17.238, 14.851),
                                        new Pose(6.365, 30.497)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                        .build();

                Path9 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(6.365, 30.497),
                                        new Pose(3.398, 21.519)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                        .build();

                Path10 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(3.398, 21.519),
                                        new Pose(3.127, 14.652)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                        .build();

                Path11 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(3.127, 14.652),
                                        new Pose(3.116, 7.751)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                        .build();

                Path12 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(3.116, 7.751),
                                        new Pose(30.812, 25.315),
                                        new Pose(60.221, 9.586)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(108))
                        .build();
            }
        }
    }
}

