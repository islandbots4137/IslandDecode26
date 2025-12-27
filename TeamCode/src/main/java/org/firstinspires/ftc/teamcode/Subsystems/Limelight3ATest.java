package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotserver.internal.webserver.controlhubupdater.result.ResultType;

import java.util.List;

public class Limelight3ATest extends OpMode {


        private Limelight3A Limelight;
        private IMU imu;

        @Override
        public void init() {
            Limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
            Limelight.pipelineSwitch(8); // Switch for

            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot revHubOrientationOnRobot =
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
            imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        }

        @Override
        public void start() {
            Limelight.start();
        }

        @Override
        public void loop() {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            Limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = Limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose_MT2();
                List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("April Tag", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                telemetry.addData("BotPose", botPose.toString());
            }
        }
}

