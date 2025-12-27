package org.firstinspires.ftc.teamcode.Mechanisms;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
@Autonomous
public class LimeLightApril extends OpMode {


    private Limelight3A Limelight;
    private IMU imu;

    @Override
    public void init() {
        Limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        Limelight.pipelineSwitch(8); // Make switch to whatever the Apriltag pipline is
    }

    @Override
    public void start() {
        Limelight.start();
    }

    @Override
    public void loop() {
        LLResult llResult = Limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("April Tag", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }

            telemetry.addData("Distance",llResult.getBotposeAvgDist());
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
        }
    }
}

