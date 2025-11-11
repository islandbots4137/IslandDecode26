package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

public class LimeLight {

    public class ArpillTag extends OpMode {

        private Limelight3A limelight;

        private IMU imu;

        @Override
        public void init() {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(8);
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot revHubOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
            imu.initialize(new IMU.Parameters(RevHubOrientationOnRobot));

        }

        @Override
        public void start() {
            limelight.start();

        }

        @Override
        public void loop() {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose();
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Tz", llResult.getTz());
                telemetry.addData("Botpose"), botpose.toString());
            }
        }

    }
}
